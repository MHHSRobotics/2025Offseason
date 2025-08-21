package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class MotorIOTalonFX extends MotorIO {
    private TalonFX motor;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private boolean configChanged = true;

    private TalonFXSimState sim;

    // Control objects
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private VoltageOut voltage = new VoltageOut(0);

    private TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0);
    private MotionMagicTorqueCurrentFOC motionMagicTorqueCurrent = new MotionMagicTorqueCurrentFOC(0);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private MotionMagicVelocityVoltage magicVelocityVoltage = new MotionMagicVelocityVoltage(0);
    private MotionMagicVelocityTorqueCurrentFOC magicVelocityTorqueCurrent = new MotionMagicVelocityTorqueCurrentFOC(0);
    private PositionVoltage positionVoltage = new PositionVoltage(0);
    private PositionTorqueCurrentFOC positionCurrent = new PositionTorqueCurrentFOC(0);
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private VelocityTorqueCurrentFOC velocityCurrent = new VelocityTorqueCurrentFOC(0);
    private Follower follow = new Follower(0, false);

    private double offset=0;

    public MotorIOTalonFX(int id, CANBus canBus) {
        motor = new TalonFX(id, canBus);
        sim = motor.getSimState();
    }

    public MotorIOTalonFX(int id, String canBus) {
        this(id, new CANBus(canBus));
    }

    public MotorIOTalonFX(int id) {
        this(id, new CANBus());
    }

    @Override
    public void updateInputs() {
        if (configChanged) {
            configChanged = false;
            motor.getConfigurator().apply(config);
        }

        // Update all the inputs from the signal values
        inputs.connected = motor.isConnected();

        // Some signals give rotations, so they have to be converted to radians
        inputs.position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble())-offset;
        inputs.velocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.accel = Units.rotationsToRadians(motor.getAcceleration().getValueAsDouble());

        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
        inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
        inputs.torqueCurrent = motor.getTorqueCurrent().getValueAsDouble();

        inputs.controlMode = motor.getControlMode().getValue().toString();

        inputs.setpoint =
                Units.rotationsToRadians(motor.getClosedLoopReference().getValueAsDouble())-offset;
        inputs.error = Units.rotationsToRadians(motor.getClosedLoopError().getValueAsDouble());
        inputs.feedforward = motor.getClosedLoopFeedForward().getValueAsDouble();
        inputs.derivOutput = motor.getClosedLoopDerivativeOutput().getValueAsDouble();
        inputs.intOutput = motor.getClosedLoopIntegratedOutput().getValueAsDouble();
        inputs.propOutput = motor.getClosedLoopProportionalOutput().getValueAsDouble();

        inputs.temp = motor.getDeviceTemp().getValueAsDouble();
        inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();

        inputs.hardwareFault = motor.getFault_Hardware().getValue();
        inputs.tempFault = motor.getFault_DeviceTemp().getValue();
        inputs.forwardLimitFault = motor.getFault_ForwardHardLimit().getValue()
                || motor.getFault_ForwardSoftLimit().getValue();
        inputs.reverseLimitFault = motor.getFault_ReverseHardLimit().getValue()
                || motor.getFault_ReverseSoftLimit().getValue();
    }

    // Sets the speed of the motor. -1 is full reverse, 1 is full forward
    @Override
    public void setSpeed(double value) {
        motor.setControl(dutyCycle.withOutput(value));
    }

    // Sets the output voltage of the motor. This is basically the same as setSpeed but scaled by 12, so -12 is full
    // reverse and 12 is full forward.
    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltage.withOutput(volts));
    }

    // Sets the torque current of the motor in amps. This is sometimes more useful than setting voltage, since it
    // automatically compensates for battery voltage and the motor's back EMF
    @Override
    public void setTorqueCurrent(double current) {
        motor.setControl(torqueCurrent.withOutput(current));
    }

    // Sets the goal of the motor using MotionMagic TorqueCurrentFOC output.
    @Override
    public void setGoalWithCurrentMagic(double position) {
        motor.setControl(motionMagicTorqueCurrent.withPosition(Radians.of(position+offset)));
    }

    // Sets the goal of the motor using MotionMagic Voltage output
    @Override
    public void setGoalWithVoltageMagic(double position) {
        motor.setControl(motionMagicVoltage.withPosition(Radians.of(position+offset)));
    }

    // Sets the goal velocity of the motor using MotionMagic TorqueCurrentFOC output.
    @Override
    public void setVelocityWithCurrentMagic(double velocity) {
        motor.setControl(magicVelocityTorqueCurrent.withVelocity(RadiansPerSecond.of(velocity)));
    }

    // Sets the goal velocity of the motor using MotionMagic Voltage output.
    @Override
    public void setVelocityWithVoltageMagic(double velocity) {
        motor.setControl(magicVelocityVoltage.withVelocity(RadiansPerSecond.of(velocity)));
    }

    // Sets the goal of the motor using TorqueCurrentFOC output.
    @Override
    public void setGoalWithCurrent(double position) {
        motor.setControl(positionCurrent.withPosition(Radians.of(position+offset)));
    }

    // Sets the goal of the motor using Voltage output
    @Override
    public void setGoalWithVoltage(double position) {
        motor.setControl(positionVoltage.withPosition(Radians.of(position+offset)));
    }

    // Sets the goal velocity of the motor using TorqueCurrentFOC output.
    @Override
    public void setVelocityWithCurrent(double velocity) {
        motor.setControl(velocityCurrent.withVelocity(RadiansPerSecond.of(velocity)));
    }

    // Sets the goal velocity of the motor using Voltage output.
    @Override
    public void setVelocityWithVoltage(double velocity) {
        motor.setControl(velocityVoltage.withVelocity(RadiansPerSecond.of(velocity)));
    }

    // Makes this motor follow another motor with the given ID. Set invert to true to follow the other motor inverted.
    // Only CTRE motors on the same CAN bus can be followed.
    @Override
    public void follow(int motorId, boolean invert) {
        motor.setControl(follow.withMasterID(motorId).withOpposeMasterDirection(invert));
    }

    // Sets whether the motor's output is inverted
    @Override
    public void setInverted(boolean inverted) {
        config.MotorOutput.Inverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        configChanged = true;
    }

    // Sets whether the motor brakes on stop
    @Override
    public void setBraking(boolean brake) {
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        configChanged = true;
    }

    // Setters for PID and feedforward tuning
    @Override
    public void setkP(double kP) {
        double newkP = Units.rotationsToRadians(kP);
        if (newkP != config.Slot0.kP) {
            config.Slot0.kP = newkP;
            configChanged = true;
        }
    }

    @Override
    public void setkI(double kI) {
        double newkI = Units.rotationsToRadians(kI);
        if (newkI != config.Slot0.kI) {
            config.Slot0.kI = newkI;
            configChanged = true;
        }
    }

    @Override
    public void setkD(double kD) {
        double newkD = Units.rotationsToRadians(kD);
        if (newkD != config.Slot0.kD) {
            config.Slot0.kD = newkD;
            configChanged = true;
        }
    }

    @Override
    public void setkS(double kS) {
        if (kS != config.Slot0.kS) {
            config.Slot0.kS = kS;
            configChanged = true;
        }
    }

    @Override
    public void setkG(double kG) {
        if (kG != config.Slot0.kG) {
            config.Slot0.kG = kG;
            configChanged = true;
        }
    }

    @Override
    public void setkV(double kV) {
        double newkV = Units.rotationsToRadians(kV);
        if (newkV != config.Slot0.kV) {
            config.Slot0.kV = newkV;
            configChanged = true;
        }
    }

    @Override
    public void setkA(double kA) {
        double newkA = Units.rotationsToRadians(kA);
        if (newkA != config.Slot0.kA) {
            config.Slot0.kA = newkA;
            configChanged = true;
        }
    }

    @Override
    public void setGains(Slot0Configs gains) {
        config.Slot0 = gains;
        configChanged = true;
    }

    @Override
    public void setMaxVelocity(double maxVelocity) {
        double newMaxVelocity = Units.radiansToRotations(maxVelocity);
        if (newMaxVelocity != config.MotionMagic.MotionMagicCruiseVelocity) {
            config.MotionMagic.MotionMagicCruiseVelocity = newMaxVelocity;
            configChanged = true;
        }
    }

    @Override
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
    @Override
    public void setContinuousWrap(boolean continuousWrap) {
        config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;
        configChanged = true;
    }

    // Sets the feedforward type for this motor. Can be either Arm_Cosine or Elevator_Static
    @Override
    public void setFeedforwardType(GravityTypeValue type) {
        config.Slot0.GravityType = type;
        configChanged = true;
    }

    // Connects a CANcoder with the given ID to this motor. motorToSensorRatio is the gear ratio between the motor and
    // encoder, sensorToMechanismRatio is the gear ratio between encoder and mechanism.
    // ONLY one of connectCANcoder and setGearRatio should be run on a given LoggedTalonFX. Use connectCANcoder if
    // you're using a CANcoder, and setGearRatio otherwise.
    @Override
    public void connectCANcoder(int id, double motorToSensorRatio, double sensorToMechanismRatio) {
        config.Feedback.FeedbackRemoteSensorID = id;
        config.Feedback.RotorToSensorRatio = motorToSensorRatio;
        config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        configChanged = true;
    }

    // Sets the gear ratio of the motor
    @Override
    public void setGearRatio(double gearRatio) {
        config.Feedback.RotorToSensorRatio = 1;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configChanged = true;
    }

    @Override
    public void setOffset(double offset) {
        this.offset=offset;
    }

    // How TalonFX current limits work: the stator current limit is the limit on how much force can be applied by the
    // motor. The supply current limit is the limit on how many amps the motor can pull from the battery. To prevent
    // brownouts, if the current pulled by the motor exceeds SupplyCurrentLowerLimit for SupplyCurrentLowerTime seconds
    // then the motor output will be clamped to SupplyCurrentLowerLimit.
    @Override
    public void setStatorCurrentLimit(double statorCurrentLimit) {
        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        configChanged = true;
    }

    @Override
    public void setSupplyCurrentLimit(double supplyCurrentLimit) {
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        configChanged = true;
    }

    @Override
    public void setSupplyCurrentLowerLimit(double supplyCurrentLowerLimit) {
        config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimit;
        configChanged = true;
    }

    @Override
    public void setSupplyCurrentLowerTime(double supplyCurrentLowerTime) {
        config.CurrentLimits.SupplyCurrentLowerTime = supplyCurrentLowerTime;
        configChanged = true;
    }

    @Override
    public void setForwardLimit(double forwardLimit) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.radiansToRotations(forwardLimit);
        configChanged = true;
    }

    @Override
    public void setReverseLimit(double reverseLimit) {
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.radiansToRotations(reverseLimit);
        configChanged = true;
    }

    @Override
    public void setMechPosition(double position) {
        sim.setRawRotorPosition(Units.radiansToRotations(
                (position+offset) * config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio));
    }

    @Override
    public void setMechVelocity(double velocity) {
        sim.setRotorVelocity(Units.radiansToRotations(
                velocity * config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio));
    }
}
