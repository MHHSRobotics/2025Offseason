package frc.robot.io;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

// Make a CTRE TalonFX-backed implementation of MotorIO.
// All position/velocity/acceleration values are doubles in mechanism units (radians for rotary, meters for linear).
// For linear mechanisms, configure the gear ratio to include spool geometry conversion.
public class MotorIOTalonFX extends MotorIO {
    private TalonFX motor;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private boolean configChanged = true;

    private TalonFXSimState sim;

    // Control objects (one per control mode)
    private NeutralOut neutral = new NeutralOut();
    private CoastOut coast = new CoastOut();
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

    private enum ControlType {
        NEUTRAL,
        COAST,
        DUTY_CYCLE,
        VOLTAGE,
        TORQUE_CURRENT,
        POS_VOLTAGE,
        POS_CURRENT,
        VEL_VOLTAGE,
        VEL_CURRENT,
        MM_POS_VOLTAGE,
        MM_POS_CURRENT,
        MM_VEL_VOLTAGE,
        MM_VEL_CURRENT,
        FOLLOW
    }

    private ControlType currentControl = ControlType.NEUTRAL;

    // Feedforward lambda
    private Supplier<Double> feedforward;

    // Whether the motor is disabled
    private boolean disabled = false;

    // Software offset (mechanism units)
    private double extraOffset;

    // Encoder connected to this motor
    private EncoderIOCANcoder connectedEncoder;

    private double minLimit = -Double.MAX_VALUE;
    private double maxLimit = Double.MAX_VALUE;

    // Whether the simulated TalonFX is disconnected
    private boolean disconnected;

    // ID and CAN bus of the motor
    private int id;
    private CANBus canBus;

    // Make a TalonFX on the given CAN bus
    public MotorIOTalonFX(int id, CANBus canBus, String name, String logPath) {
        super(name, logPath);
        motor = new TalonFX(id, canBus);
        sim = motor.getSimState();
        this.id = id;
        this.canBus = canBus;
    }

    // Make a TalonFX on a named CAN bus (e.g., "rio", "canivore")
    public MotorIOTalonFX(int id, String canBus, String name, String logPath) {
        this(id, new CANBus(canBus), name, logPath);
    }

    // Make a TalonFX on the default CAN bus
    public MotorIOTalonFX(int id, String name, String logPath) {
        this(id, new CANBus(), name, logPath);
    }

    public int getId() {
        return id;
    }

    public CANBus getCANBus() {
        return canBus;
    }

    @Override
    public void update() {
        if (configChanged) {
            configChanged = false;
            motor.getConfigurator().apply(config);
        }

        if (disabled) {
            currentControl = ControlType.NEUTRAL;
        }

        // Update all input values from the motor signals
        inputs.connected = disconnected ? false : motor.isConnected();

        // Convert rotations to radians for mechanism units
        inputs.positionRad = Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) - extraOffset;
        inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.accelRadPerSecSquared = Units.rotationsToRadians(motor.getAcceleration().getValueAsDouble());

        inputs.appliedVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyVoltageVolts = motor.getSupplyVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.torqueCurrentAmps = motor.getTorqueCurrent().getValueAsDouble();

        inputs.controlMode = currentControl.name();

        // Get the setpoint
        double setpoint = Units.rotationsToRadians(motor.getClosedLoopReference().getValueAsDouble());
        switch (currentControl) {
            case COAST, NEUTRAL, FOLLOW, VOLTAGE, DUTY_CYCLE, TORQUE_CURRENT:
                // If PID control is disabled, log 0 setpoint
                inputs.setpoint = 0;
                break;
            case VEL_CURRENT, VEL_VOLTAGE, MM_VEL_CURRENT, MM_VEL_VOLTAGE:
                // If velocity PID control is enabled, log setpoint
                inputs.setpoint = setpoint;
                break;
            default:
                // For position control modes, subtract the extra offset
                inputs.setpoint = setpoint - extraOffset;
                break;
        }

        inputs.setpointVelocity = Units.rotationsToRadians(motor.getClosedLoopReferenceSlope().getValueAsDouble());

        inputs.error = Units.rotationsToRadians(motor.getClosedLoopError().getValueAsDouble());
        inputs.feedforward = motor.getClosedLoopFeedForward().getValueAsDouble();
        inputs.derivOutput = motor.getClosedLoopDerivativeOutput().getValueAsDouble();
        inputs.intOutput = motor.getClosedLoopIntegratedOutput().getValueAsDouble();
        inputs.propOutput = motor.getClosedLoopProportionalOutput().getValueAsDouble();

        inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
        inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();

        inputs.hardwareFault = motor.getFault_Hardware().getValue();
        inputs.tempFault = motor.getFault_DeviceTemp().getValue();
        inputs.forwardLimitFault =
                motor.getFault_ForwardHardLimit().getValue() || motor.getFault_ForwardSoftLimit().getValue();
        inputs.reverseLimitFault =
                motor.getFault_ReverseHardLimit().getValue() || motor.getFault_ReverseSoftLimit().getValue();

        inputs.rotorPositionRad = Units.rotationsToRadians(motor.getRotorPosition().getValueAsDouble()
                / (config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio));

        if (connectedEncoder != null) {
            inputs.encoderDiffRad = inputs.positionRad - connectedEncoder.getInputs().positionRad;
        }

        // Update alerts using the base class method (this checks all fault conditions and updates dashboard alerts)
        super.update();

        // Current feedforward given by FF lambda
        double currentFeedforward = feedforward == null ? 0 : feedforward.get();

        // Set motor control
        switch (currentControl) {
            case NEUTRAL:
                motor.setControl(neutral);
                break;
            case COAST:
                motor.setControl(coast);
                break;
            case VOLTAGE:
                motor.setControl(voltage);
                break;
            case DUTY_CYCLE:
                motor.setControl(dutyCycle);
                break;
            case TORQUE_CURRENT:
                motor.setControl(torqueCurrent);
                break;
            case POS_CURRENT:
                motor.setControl(positionCurrent.withFeedForward(currentFeedforward));
                break;
            case POS_VOLTAGE:
                motor.setControl(positionVoltage.withFeedForward(currentFeedforward));
                break;
            case VEL_CURRENT:
                motor.setControl(velocityCurrent.withFeedForward(currentFeedforward));
                break;
            case VEL_VOLTAGE:
                motor.setControl(velocityVoltage.withFeedForward(currentFeedforward));
                break;
            case MM_POS_CURRENT:
                motor.setControl(motionMagicTorqueCurrent.withFeedForward(currentFeedforward));
                break;
            case MM_POS_VOLTAGE:
                motor.setControl(motionMagicVoltage.withFeedForward(currentFeedforward));
                break;
            case MM_VEL_CURRENT:
                motor.setControl(magicVelocityTorqueCurrent.withFeedForward(currentFeedforward));
                break;
            case MM_VEL_VOLTAGE:
                motor.setControl(magicVelocityVoltage.withFeedForward(currentFeedforward));
                break;
            case FOLLOW:
                motor.setControl(follow);
                break;
        }
    }

    // Tell the motor how fast to spin (percent, -1 = full reverse, 1 = full forward)
    @Override
    public void setDutyCycle(double value) {
        dutyCycle.withOutput(value);
        currentControl = ControlType.DUTY_CYCLE;
    }

    // Tell the motor what voltage to apply (volts)
    @Override
    public void setVoltage(double volts) {
        voltage.withOutput(volts);
        currentControl = ControlType.VOLTAGE;
    }

    // Tell the motor the torque-producing current to use (amps)
    @Override
    public void setTorqueCurrent(double amps) {
        torqueCurrent.withOutput(amps);
        currentControl = ControlType.TORQUE_CURRENT;
    }

    // Tell the motor to go to a target position using Motion Magic with current control (mechanism units)
    @Override
    public void setGoalWithCurrentMagic(double goal, Supplier<Double> feedforward) {
        goal = MathUtil.clamp(goal, minLimit, maxLimit);
        motionMagicTorqueCurrent.withPosition(Units.radiansToRotations(goal + extraOffset));
        currentControl = ControlType.MM_POS_CURRENT;
        this.feedforward = feedforward;
    }

    // Tell the motor to go to a target position using Motion Magic with voltage control (mechanism units)
    @Override
    public void setGoalWithVoltageMagic(double goal, Supplier<Double> feedforward) {
        goal = MathUtil.clamp(goal, minLimit, maxLimit);
        motionMagicVoltage.withPosition(Units.radiansToRotations(goal + extraOffset));
        currentControl = ControlType.MM_POS_VOLTAGE;
        this.feedforward = feedforward;
    }

    // Tell the motor to reach a target speed using Motion Magic with current control (mechanism units/s)
    @Override
    public void setVelocityWithCurrentMagic(double velocity, Supplier<Double> feedforward) {
        magicVelocityTorqueCurrent.withVelocity(Units.radiansToRotations(velocity));
        currentControl = ControlType.MM_VEL_CURRENT;
        this.feedforward = feedforward;
    }

    // Tell the motor to reach a target speed using Motion Magic with voltage control (mechanism units/s)
    @Override
    public void setVelocityWithVoltageMagic(double velocity, Supplier<Double> feedforward) {
        magicVelocityVoltage.withVelocity(Units.radiansToRotations(velocity));
        currentControl = ControlType.MM_VEL_VOLTAGE;
        this.feedforward = feedforward;
    }

    // Tell the motor to go to a target position using current control (mechanism units)
    @Override
    public void setGoalWithCurrent(double goal, Supplier<Double> feedforward) {
        goal = MathUtil.clamp(goal, minLimit, maxLimit);
        positionCurrent.withPosition(Units.radiansToRotations(goal + extraOffset));
        currentControl = ControlType.POS_CURRENT;
        this.feedforward = feedforward;
    }

    // Tell the motor to go to a target position using voltage control (mechanism units)
    @Override
    public void setGoalWithVoltage(double goal, Supplier<Double> feedforward) {
        goal = MathUtil.clamp(goal, minLimit, maxLimit);
        positionVoltage.withPosition(Units.radiansToRotations(goal + extraOffset));
        currentControl = ControlType.POS_VOLTAGE;
        this.feedforward = feedforward;
    }

    // Tell the motor to reach a target speed using current control (mechanism units/s)
    @Override
    public void setVelocityWithCurrent(double velocity, Supplier<Double> feedforward) {
        velocityCurrent.withVelocity(Units.radiansToRotations(velocity));
        currentControl = ControlType.VEL_CURRENT;
        this.feedforward = feedforward;
    }

    // Tell the motor to reach a target speed using voltage control (mechanism units/s)
    @Override
    public void setVelocityWithVoltage(double velocity, Supplier<Double> feedforward) {
        velocityVoltage.withVelocity(Units.radiansToRotations(velocity));
        currentControl = ControlType.VEL_VOLTAGE;
        this.feedforward = feedforward;
    }

    // Make this motor follow another motor with the given CAN ID (invert if needed).
    // Note: Only CTRE motors on the same CAN bus can be followed.
    @Override
    public void follow(MotorIO motor, boolean invert) {
        if (motor instanceof MotorIOTalonFX talon) {
            follow.withMasterID(talon.getId()).withOpposeMasterDirection(invert);
            currentControl = ControlType.FOLLOW;
        } else {
            Alerts.create(
                    "TalonFX " + getName() + " doesn't support following motors other than TalonFX's",
                    AlertType.kError);
        }
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

    // Modifies gains for unit scaling
    @Override
    public void setGains(Slot0Configs gains) {
        setkP(gains.kP);
        setkI(gains.kI);
        setkD(gains.kD);
        setkG(gains.kG);
        setkS(gains.kS);
        setkV(gains.kV);
        setkA(gains.kA);
        setFeedforwardType(gains.GravityType);
        setStaticFeedforwardSign(gains.StaticFeedforwardSign);
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
        config.ClosedLoopGeneral.ContinuousWrap = continuousWrap;
        configChanged = true;
    }

    // Tell the controller which gravity model to use (Arm_Cosine or Elevator_Static)
    @Override
    public void setFeedforwardType(GravityTypeValue type) {
        config.Slot0.GravityType = type;
        configChanged = true;
    }

    // Tell the controller which sign to use for kS (closed loop sign or velocity sign)
    @Override
    public void setStaticFeedforwardSign(StaticFeedforwardSignValue feedforwardSign) {
        config.Slot0.StaticFeedforwardSign = feedforwardSign;
        configChanged = true;
    }

    // Tell the motor to use a remote encoder with gear ratios:
    // - motorToSensorRatio: motor rotations to sensor rotations (unitless)
    // - fuse: Whether to use the internal rotor along with the CANcoder. Always set to true, unless there are issues
    // with the reported position teleporting even after accounting for gear ratio and inversion, in which case it
    // should be false
    // Only use ONE of connectEncoder OR setGearRatio for a motor, not both.
    // Currently only supports CANcoders.
    @Override
    public void connectEncoder(EncoderIO encoder, double motorToSensorRatio, boolean fuse) {
        if (encoder instanceof EncoderIOCANcoder cancoder) {
            config.Feedback.FeedbackRemoteSensorID = cancoder.getId();
            config.Feedback.FeedbackSensorSource =
                    fuse ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RemoteCANcoder;
            config.Feedback.RotorToSensorRatio = motorToSensorRatio;
            config.Feedback.SensorToMechanismRatio = cancoder.getRatio();
            connectedEncoder = cancoder;
            configChanged = true;
        } else {
            Alerts.create(
                    "TalonFX " + getName() + " doesn't support feedback sources other than CANcoders",
                    AlertType.kError);
        }
    }

    // Tell the motor to use its internal sensor with a gear ratio to the mechanism
    @Override
    public void setGearRatio(double motorToMechanismRatio) {
        config.Feedback.RotorToSensorRatio = 1;
        config.Feedback.SensorToMechanismRatio = motorToMechanismRatio;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        configChanged = true;
    }

    // Use after connectEncoder/setGearRatio. Sets the mechanism offset (mechanism units).
    @Override
    public void setOffset(double offset) {
        if (config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.FusedCANcoder
                || config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RemoteCANcoder) {
            connectedEncoder.setOffset(offset);
            extraOffset = connectedEncoder.getExtraOffset();
        } else if (config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RotorSensor) {
            double ratio = config.Feedback.SensorToMechanismRatio;

            // Convert mechanism offset to mech rotations
            double rotOffset = Units.radiansToRotations(offset);

            // Wrap to [-0.5, 0.5] range to find the fractional rotation part
            double remOffset = rotOffset - Math.round(rotOffset);

            // Actual offset required
            double rotorOffset = remOffset * ratio;

            if (Math.abs(rotorOffset) <= 1) {
                config.Feedback.FeedbackRotorOffset = rotorOffset;

                extraOffset = Units.rotationsToRadians(rotOffset - remOffset);
            } else {
                // FALLBACK CASE: If we can't fit in MagnetOffset, put entire offset in software
                config.Feedback.FeedbackRotorOffset = 0;
                extraOffset = offset;

                // Warn because non-2π multiples in extraOffset break gravity compensation assumptions
                Alerts.create(
                        "extraOffset is not a multiple of 2pi--if " + getName()
                                + " is used in an arm mechanism, kG will not account for gravity correctly",
                        AlertType.kWarning);
            }
            configChanged = true;
        } else {
            Alerts.create("Invalid sensor source for TalonFX " + getName(), AlertType.kError);
        }
    }

    // Current limits:
    // - StatorCurrentLimit: limit on torque-producing current (amps)
    // - SupplyCurrentLimit: limit on battery current draw (amps)
    // - If current > SupplyCurrentLowerLimit for SupplyCurrentLowerTime seconds, clamp to SupplyCurrentLowerLimit
    @Override
    public void setStatorCurrentLimit(double amps) {
        config.CurrentLimits.withStatorCurrentLimit(amps);
        configChanged = true;
    }

    @Override
    public void setSupplyCurrentLimit(double amps) {
        config.CurrentLimits.withSupplyCurrentLimit(amps);
        configChanged = true;
    }

    @Override
    public void setSupplyCurrentLowerLimit(double amps) {
        config.CurrentLimits.withSupplyCurrentLowerLimit(amps);
        configChanged = true;
    }

    @Override
    public void setSupplyCurrentLowerTime(double seconds) {
        config.CurrentLimits.withSupplyCurrentLowerTime(seconds);
        configChanged = true;
    }

    @Override
    public void setLimits(double min, double max) {
        minLimit = min;
        maxLimit = max;
        config.SoftwareLimitSwitch
                .withForwardSoftLimitThreshold(Units.radiansToRotations(max + extraOffset))
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Units.radiansToRotations(min + extraOffset))
                .withReverseSoftLimitEnable(true);
        configChanged = true;
    }

    // Disables all motor output
    @Override
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    // We apply invert after adding offset because invert is applied before offset in the position reading code
    @Override
    public void setMechPosition(double position) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechPosition on " + getName(), AlertType.kWarning);
            return;
        }
        double rotorPos = Units.radiansToRotations(position + extraOffset)
                * (config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio);
        if (config.Feedback.FeedbackSensorSource == FeedbackSensorSourceValue.RotorSensor) {
            rotorPos += config.Feedback.FeedbackRotorOffset;
        }
        rotorPos = config.MotorOutput.Inverted.equals(InvertedValue.Clockwise_Positive) ? -rotorPos : rotorPos;
        sim.setRawRotorPosition(rotorPos);
    }

    @Override
    public void setMechVelocity(double velocity) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setMechVelocity on " + getName(), AlertType.kWarning);
            return;
        }
        double rotorVel = Units.radiansToRotations(velocity)
                * (config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio);
        rotorVel = config.MotorOutput.Inverted.equals(InvertedValue.Clockwise_Positive) ? -rotorVel : rotorVel;
        sim.setRotorVelocity(rotorVel);
    }

    @Override
    public void setConnected(boolean connected) {
        if (Constants.currentMode == Mode.REAL) {
            Alerts.create("Used sim-only method setConnected on " + getName(), AlertType.kWarning);
            return;
        }
        disconnected = !connected;
    }
}
