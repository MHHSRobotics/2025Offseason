package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
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

// Make a CTRE TalonFX-backed implementation of MotorIO.
// Units used:
// - Mechanism position in radians (rad) for arms/flywheels, meters (m) for elevators
// - Speeds in rad/s or m/s
// - Voltages in volts, currents in amps
public class MotorIOTalonFX extends MotorIO {
    private TalonFX motor;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private boolean configChanged = true;

    private TalonFXSimState sim;

    // Control objects (one per control mode)
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

    // Current offset of the motor
    private double offset = 0;

    // Make a TalonFX on the given CAN bus
    public MotorIOTalonFX(int id, CANBus canBus) {
        motor = new TalonFX(id, canBus);
        sim = motor.getSimState();
    }

    // Make a TalonFX on a named CAN bus (e.g., "rio", "canivore")
    public MotorIOTalonFX(int id, String canBus) {
        this(id, new CANBus(canBus));
    }

    // Make a TalonFX on the default CAN bus
    public MotorIOTalonFX(int id) {
        this(id, new CANBus());
    }

    @Override
    public void update() {
        if (configChanged) {
            configChanged = false;
            ControlRequest control = motor.getAppliedControl();
            motor.getConfigurator().apply(config);
            motor.setControl(control);
        }

        // Update all input values from the motor signals
        inputs.connected = motor.isConnected();

        // Convert rotations to radians for mechanism units
        inputs.position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) - offset;
        inputs.velocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.accel = Units.rotationsToRadians(motor.getAcceleration().getValueAsDouble());

        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
        inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
        inputs.torqueCurrent = motor.getTorqueCurrent().getValueAsDouble();

        inputs.controlMode = motor.getControlMode().getValue().toString();

        inputs.setpoint =
                Units.rotationsToRadians(motor.getClosedLoopReference().getValueAsDouble()) - offset;
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

        // Update alerts using the base class method (this checks all fault conditions and updates dashboard alerts)
        super.update();
    }

    // Tell the motor how fast to spin (percent, -1 = full reverse, 1 = full forward)
    @Override
    public void setSpeed(double value) {
        motor.setControl(dutyCycle.withOutput(value));
    }

    // Tell the motor what voltage to apply (volts). Similar to setSpeed but in volts.
    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltage.withOutput(volts));
    }

    // Tell the motor the torque-producing current to use (amps). Helpful to ignore battery sag and back-EMF.
    @Override
    public void setTorqueCurrent(double current) {
        motor.setControl(torqueCurrent.withOutput(current));
    }

    // Tell the motor to go to a target position using Motion Magic with current control (radians)
    @Override
    public void setGoalWithCurrentMagic(double position) {
        motor.setControl(motionMagicTorqueCurrent.withPosition(Radians.of(position + offset)));
    }

    // Tell the motor to go to a target position using Motion Magic with voltage control (radians)
    @Override
    public void setGoalWithVoltageMagic(double position) {
        motor.setControl(motionMagicVoltage.withPosition(Radians.of(position + offset)));
    }

    // Tell the motor to reach a target speed using Motion Magic with current control (rad/s)
    @Override
    public void setVelocityWithCurrentMagic(double velocity) {
        motor.setControl(magicVelocityTorqueCurrent.withVelocity(RadiansPerSecond.of(velocity)));
    }

    // Tell the motor to reach a target speed using Motion Magic with voltage control (rad/s)
    @Override
    public void setVelocityWithVoltageMagic(double velocity) {
        motor.setControl(magicVelocityVoltage.withVelocity(RadiansPerSecond.of(velocity)));
    }

    // Tell the motor to go to a target position using current control (radians)
    @Override
    public void setGoalWithCurrent(double position) {
        motor.setControl(positionCurrent.withPosition(Radians.of(position + offset)));
    }

    // Tell the motor to go to a target position using voltage control (radians)
    @Override
    public void setGoalWithVoltage(double position) {
        motor.setControl(positionVoltage.withPosition(Radians.of(position + offset)));
    }

    // Tell the motor to reach a target speed using current control (rad/s)
    @Override
    public void setVelocityWithCurrent(double velocity) {
        motor.setControl(velocityCurrent.withVelocity(RadiansPerSecond.of(velocity)));
    }

    // Tell the motor to reach a target speed using voltage control (rad/s)
    @Override
    public void setVelocityWithVoltage(double velocity) {
        motor.setControl(velocityVoltage.withVelocity(RadiansPerSecond.of(velocity)));
    }

    // Make this motor follow another motor with the given CAN ID (invert if needed).
    // Note: Only CTRE motors on the same CAN bus can be followed.
    @Override
    public void follow(int motorId, boolean invert) {
        motor.setControl(follow.withMasterID(motorId).withOpposeMasterDirection(invert));
    }

    // Tell the motor which direction is forward (true = invert)
    @Override
    public void setInverted(boolean inverted) {
        InvertedValue newInverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        if (newInverted != config.MotorOutput.Inverted) {
            config.MotorOutput.Inverted = newInverted;
            configChanged = true;
        }
    }

    // Tell the motor what to do when stopped: brake (hold) or coast (freewheel)
    @Override
    public void setBraking(boolean brake) {
        NeutralModeValue newNeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        if (newNeutralMode != config.MotorOutput.NeutralMode) {
            config.MotorOutput.NeutralMode = newNeutralMode;
            configChanged = true;
        }
    }

    // Make PID and feedforward values active (converting from rotations-based to radians-based where needed)
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

    @Override
    public void setMaxJerk(double maxJerk) {
        double newMaxJerk = Units.radiansToRotations(maxJerk);
        if (newMaxJerk != config.MotionMagic.MotionMagicJerk) {
            config.MotionMagic.MotionMagicJerk = newMaxJerk;
            configChanged = true;
        }
    }

    // Make continuous wrap enabled for mechanisms that can spin > 360° (like swerve azimuth)
    @Override
    public void setContinuousWrap(boolean continuousWrap) {
        if (continuousWrap != config.ClosedLoopGeneral.ContinuousWrap) {
            config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;
            configChanged = true;
        }
    }

    // Tell the controller which gravity model to use (Arm_Cosine or Elevator_Static)
    @Override
    public void setFeedforwardType(GravityTypeValue type) {
        if (type != config.Slot0.GravityType) {
            config.Slot0.GravityType = type;
            configChanged = true;
        }
    }

    // Tell the motor to use a remote CANcoder (id) with gear ratios:
    // - motorToSensorRatio: motor rotations to sensor rotations (unitless)
    // - sensorToMechanismRatio: sensor rotations to mechanism rotations (unitless)
    // Only use ONE of connectCANcoder OR setGearRatio for a motor, not both.
    @Override
    public void connectCANcoder(int id, double motorToSensorRatio, double sensorToMechanismRatio) {
        if (id != config.Feedback.FeedbackRemoteSensorID
                || motorToSensorRatio != config.Feedback.RotorToSensorRatio
                || sensorToMechanismRatio != config.Feedback.SensorToMechanismRatio
                || config.Feedback.FeedbackSensorSource != FeedbackSensorSourceValue.FusedCANcoder) {
            config.Feedback.FeedbackRemoteSensorID = id;
            config.Feedback.RotorToSensorRatio = motorToSensorRatio;
            config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            configChanged = true;
        }
    }

    // Tell the motor to use its internal sensor with a gear ratio to the mechanism (unitless)
    @Override
    public void setGearRatio(double motorToMechanismRatio) {
        if (1.0 != config.Feedback.RotorToSensorRatio
                || motorToMechanismRatio != config.Feedback.SensorToMechanismRatio
                || config.Feedback.FeedbackSensorSource != FeedbackSensorSourceValue.RotorSensor) {
            config.Feedback.RotorToSensorRatio = 1;
            config.Feedback.SensorToMechanismRatio = motorToMechanismRatio;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            configChanged = true;
        }
    }

    // Set the offset of this motor, or what it reports at the 0 position. This will be subtracted from the reported
    // position. Make sure to call this before setting limits!
    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }

    // Current limits:
    // - StatorCurrentLimit: limit on torque-producing current (amps)
    // - SupplyCurrentLimit: limit on battery current draw (amps)
    // - If current > SupplyCurrentLowerLimit for SupplyCurrentLowerTime seconds, clamp to SupplyCurrentLowerLimit
    @Override
    public void setStatorCurrentLimit(double statorCurrentLimit) {
        if (statorCurrentLimit != config.CurrentLimits.StatorCurrentLimit) {
            config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
            configChanged = true;
        }
    }

    @Override
    public void setSupplyCurrentLimit(double supplyCurrentLimit) {
        if (supplyCurrentLimit != config.CurrentLimits.SupplyCurrentLimit) {
            config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
            configChanged = true;
        }
    }

    @Override
    public void setSupplyCurrentLowerLimit(double supplyCurrentLowerLimit) {
        if (supplyCurrentLowerLimit != config.CurrentLimits.SupplyCurrentLowerLimit) {
            config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimit;
            configChanged = true;
        }
    }

    @Override
    public void setSupplyCurrentLowerTime(double supplyCurrentLowerTime) {
        if (supplyCurrentLowerTime != config.CurrentLimits.SupplyCurrentLowerTime) {
            config.CurrentLimits.SupplyCurrentLowerTime = supplyCurrentLowerTime;
            configChanged = true;
        }
    }

    @Override
    public void setLimits(double min, double max) {
        double newForwardThreshold = Units.radiansToRotations(max + offset);
        double newReverseThreshold = Units.radiansToRotations(min + offset);
        if (!config.SoftwareLimitSwitch.ForwardSoftLimitEnable
                || newForwardThreshold != config.SoftwareLimitSwitch.ForwardSoftLimitThreshold
                || !config.SoftwareLimitSwitch.ReverseSoftLimitEnable
                || newReverseThreshold != config.SoftwareLimitSwitch.ReverseSoftLimitThreshold) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = newForwardThreshold;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = newReverseThreshold;
            configChanged = true;
        }
    }

    // We apply invert after adding offset because invert is applied before offset in the position reading code
    @Override
    public void setMechPosition(double position) {
        double rotorPos = Units.radiansToRotations(
                (position + offset) * config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio);
        rotorPos = config.MotorOutput.Inverted.equals(InvertedValue.Clockwise_Positive) ? -rotorPos : rotorPos;
        sim.setRawRotorPosition(rotorPos);
    }

    @Override
    public void setMechVelocity(double velocity) {
        double rotorVel = Units.radiansToRotations(
                velocity * config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio);
        rotorVel = config.MotorOutput.Inverted.equals(InvertedValue.Clockwise_Positive) ? -rotorVel : rotorVel;
        sim.setRotorVelocity(rotorVel);
    }

    @Override
    public void clearStickyFaults() {
        motor.clearStickyFaults();
    }
}
