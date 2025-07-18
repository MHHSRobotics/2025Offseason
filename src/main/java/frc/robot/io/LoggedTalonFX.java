package frc.robot.io;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.TalonFXIO.TalonFXIOInputs;

import static edu.wpi.first.units.Units.Radians;

// Manages alerts, logging, and control of a TalonFXIO
public class LoggedTalonFX {
    // NetworkTables path to log to
    private String logPath;

    // The TalonFX interface
    private final TalonFXIO io;

    // Current inputs from the TalonFXIO
    private final TalonFXIOInputsAutoLogged inputs = new TalonFXIOInputsAutoLogged();

    // Control objects
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private VoltageOut voltage = new VoltageOut(0);
    private MotionMagicTorqueCurrentFOC motionMagicTorqueCurrent = new MotionMagicTorqueCurrentFOC(0);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0);
    private Follower follow = new Follower(0, false);

    // Alerts for faults. These will appear on AdvantageScope/Elastic
    private Alert disconnectedAlert;

    private Alert hardwareAlert;
    private Alert procTempAlert;
    private Alert deviceTempAlert;
    private Alert undervoltageAlert;
    private Alert bootDuringEnableAlert;
    private Alert forwardHardLimitAlert;
    private Alert forwardSoftLimitAlert;
    private Alert reverseHardLimitAlert;
    private Alert reverseSoftLimitAlert;

    // Current TalonFX config
    private TalonFXConfiguration config;

    // Whether the config has changed after the last update
    private boolean configChanged = true;

    public LoggedTalonFX(TalonFXIO io, String logPath) {
        this.io = io;
        this.logPath = logPath;

        // Create an empty config
        config = new TalonFXConfiguration();

        // Initialize alerts
        disconnectedAlert = new Alert(logPath + " is disconnected", AlertType.kError);

        hardwareAlert = new Alert(logPath + " encountered a hardware fault", AlertType.kWarning);
        procTempAlert = new Alert(logPath + " processor is overheating", AlertType.kWarning);
        deviceTempAlert = new Alert(logPath + " is overheating", AlertType.kWarning);
        undervoltageAlert = new Alert(logPath + " has insufficient voltage", AlertType.kWarning);
        bootDuringEnableAlert = new Alert(logPath + " booted during enable", AlertType.kWarning);
        forwardHardLimitAlert = new Alert(logPath + " reached its forward hard limit", AlertType.kWarning);
        forwardSoftLimitAlert = new Alert(logPath + " reached its forward soft limit", AlertType.kWarning);
        reverseHardLimitAlert = new Alert(logPath + " reached its reverse hard limit", AlertType.kWarning);
        reverseSoftLimitAlert = new Alert(logPath + " reached its reverse soft limit", AlertType.kWarning);
    }

    public void periodic() {
        // Get the new inputs from TalonFXIO and log them
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);

        // Display any alerts that are currently active
        disconnectedAlert.set(!inputs.connected);

        hardwareAlert.set(inputs.hardwareFault);
        procTempAlert.set(inputs.procTempFault);
        deviceTempAlert.set(inputs.deviceTempFault);
        undervoltageAlert.set(inputs.undervoltageFault);
        bootDuringEnableAlert.set(inputs.bootDuringEnable);
        forwardHardLimitAlert.set(inputs.forwardHardLimit);
        forwardSoftLimitAlert.set(inputs.forwardSoftLimit);
        reverseHardLimitAlert.set(inputs.reverseHardLimit);
        reverseSoftLimitAlert.set(inputs.reverseSoftLimit);

        // If the config has changed in the last frame, apply it. We do a lot of changes in one frame during
        // initialization so this batches the operations.
        if (configChanged) {
            io.applyConfig(config);
            configChanged = false;
        }
    }

    // Returns the position of the motor in mechanism units
    public double getPosition() {
        return inputs.position;
    }

    // Returns the velocity of the motor in mechanism units/s
    public double getVelocity() {
        return inputs.velocity;
    }

    // Returns the current goal of the motor in mechanism units
    public double getGoal() {
        return inputs.setpoint;
    }

    // Gets the current input object. Use this to get all the motor info that isn't given by the above three getters.
    public TalonFXIOInputs getInputs() {
        return inputs;
    }

    // Sets the speed of the motor. -1 is full reverse, 1 is full forward
    public void setSpeed(double value) {
        io.setControl(dutyCycle.withOutput(value));
    }

    // Sets the output voltage of the motor. This is basically the same as setSpeed but scaled by 12, so -12 is full
    // reverse and 12 is full forward.
    public void setVoltage(double volts) {
        io.setControl(voltage.withOutput(volts));
    }

    // Sets the torque current of the motor in amps. This is sometimes more useful than setting voltage, since it
    // automatically compensates for battery voltage and the motor's back EMF
    public void setTorqueCurrent(double current) {
        io.setControl(torqueCurrent.withOutput(current));
    }

    // Sets the goal of the motor using MotionMagic TorqueCurrentFOC output. Don't use this, use setGoalWithVoltage.
    // TorqueCUrrentFOC can't be characterized with SysId.
    public void setGoalWithCurrent(double position) {
        io.setControl(motionMagicTorqueCurrent.withPosition(Radians.of(position)));
    }

    // Sets the goal of the motor using MotionMagic Voltage output
    public void setGoalWithVoltage(double position) {
        io.setControl(motionMagicVoltage.withPosition(Radians.of(position)));
    }

    // Makes this motor follow another motor with the given ID. Set invert to true to follow the other motor inverted.
    // Only CTRE motors on the same CAN bus can be followed.
    public void follow(int motorId, boolean invert) {
        io.setControl(follow.withMasterID(motorId).withOpposeMasterDirection(invert));
    }

    // Sets whether the motor's output is inverted
    public void setInverted(boolean inverted) {
        config.MotorOutput.Inverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        configChanged = true;
    }

    // Sets whether the motor brakes on stop
    public void setBraking(boolean brake) {
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        configChanged = true;
    }

    // Setters for PID and feedforward tuning
    public void setkP(double kP) {
        double newkP = Units.rotationsToRadians(kP);
        if (newkP != config.Slot0.kP) {
            config.Slot0.kP = newkP;
            configChanged = true;
        }
    }

    public void setkI(double kI) {
        double newkI = Units.rotationsToRadians(kI);
        if (newkI != config.Slot0.kI) {
            config.Slot0.kI = newkI;
            configChanged = true;
        }
    }

    public void setkD(double kD) {
        double newkD = Units.rotationsToRadians(kD);
        if (newkD != config.Slot0.kD) {
            config.Slot0.kD = newkD;
            configChanged = true;
        }
    }

    public void setkS(double kS) {
        if (kS != config.Slot0.kS) {
            config.Slot0.kS = kS;
            configChanged = true;
        }
    }

    public void setkG(double kG) {
        if (kG != config.Slot0.kG) {
            config.Slot0.kG = kG;
            configChanged = true;
        }
    }

    public void setkV(double kV) {
        double newkV = Units.rotationsToRadians(kV);
        if (newkV != config.Slot0.kV) {
            config.Slot0.kV = newkV;
            configChanged = true;
        }
    }

    public void setkA(double kA) {
        double newkA = Units.rotationsToRadians(kA);
        if (newkA != config.Slot0.kA) {
            config.Slot0.kA = newkA;
            configChanged = true;
        }
    }

    public void setMaxVelocity(double maxVelocity) {
        double newMaxVelocity = Units.radiansToRotations(maxVelocity);
        if (newMaxVelocity != config.MotionMagic.MotionMagicCruiseVelocity) {
            config.MotionMagic.MotionMagicCruiseVelocity = newMaxVelocity;
            configChanged = true;
        }
    }

    public void setMaxAccel(double maxAccel) {
        double newMaxAccel = Units.radiansToRotations(maxAccel);
        if (newMaxAccel != config.MotionMagic.MotionMagicAcceleration) {
            config.MotionMagic.MotionMagicAcceleration = newMaxAccel;
            configChanged = true;
        }
    }

    // Sets whether continuous wrap should be enabled for the motor. This basically tells the TalonFX that it's attached
    // to a mechanism that can go the full 360 degrees, so it can move in either direction to reach its goal. The swerve
    // angle motors use this.
    public void setContinuousWrap(boolean continuousWrap) {
        config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;
        configChanged = true;
    }

    // Sets the feedforward type for this motor. Can be either Arm_Cosine or Elevator_Static
    public void setFeedforwardType(GravityTypeValue type) {
        config.Slot0.GravityType = type;
        configChanged = true;
    }

    // Connects a CANcoder with the given ID to this motor. motorToSensorRatio is the gear ratio between the motor and
    // encoder, sensorToMechanismRatio is the gear ratio between encoder and mechanism.
    // ONLY one of connectCANcoder and setGearRatio should be run on a given LoggedTalonFX. Use connectCANcoder if
    // you're using a CANcoder, and setGearRatio otherwise.
    public void connectCANcoder(int id, double motorToSensorRatio, double sensorToMechanismRatio) {
        config.Feedback.FeedbackRemoteSensorID = id;
        config.Feedback.RotorToSensorRatio = motorToSensorRatio;
        config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        configChanged = true;
    }

    // Sets the gear ratio of the motor
    public void setGearRatio(double gearRatio) {
        config.Feedback.RotorToSensorRatio = 1;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configChanged = true;
    }

    // How TalonFX current limits work: the stator current limit is the limit on how much force can be applied by the
    // motor. The supply current limit is the limit on how many amps the motor can pull from the battery. To prevent
    // brownouts, if the current pulled by the motor exceeds SupplyCurrentLowerLimit for SupplyCurrentLowerTime seconds
    // then the motor output will be clamped to SupplyCurrentLowerLimit.
    public void setStatorCurrentLimit(double statorCurrentLimit) {
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        configChanged = true;
    }

    public void setSupplyCurrentLimit(double supplyCurrentLimit) {
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        configChanged = true;
    }

    public void setSupplyCurrentLowerLimit(double supplyCurrentLowerLimit) {
        config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimit;
        configChanged = true;
    }

    public void setSupplyCurrentLowerTime(double supplyCurrentLowerTime) {
        config.CurrentLimits.SupplyCurrentLowerTime = supplyCurrentLowerTime;
        configChanged = true;
    }
}
