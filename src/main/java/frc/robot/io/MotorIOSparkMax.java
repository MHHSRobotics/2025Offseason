package frc.robot.io;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Make a REV SparkMax-backed implementation of MotorIO.
// Units used:
// - Mechanism position in radians (rad) for arms/flywheels, meters (m) for elevators
// - Speeds in rad/s or m/s
// - Voltages in volts, currents in amps
// Currently some features are missing, like connecting to an external encoder.
public class MotorIOSparkMax extends MotorIO {
    private SparkMax motor;
    private SparkMaxSim sim;
    private SparkMaxConfig config = new SparkMaxConfig();
    private boolean configChanged = true;
    
    private RelativeEncoder encoder;

    // On-RIO control systems (much better than SparkMax built-in)
    private PIDController pidController = new PIDController(0, 0, 0);
    private ProfiledPIDController profiledPidController;
    private SimpleMotorFeedforward simpleFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
    private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 0, 0);
    
    // Control state
    private ControlMode currentControlMode = ControlMode.NONE;
    private double currentSetpoint = 0;
    private boolean continuousWrap = false;
    private GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;
    private double lastTime = Timer.getFPGATimestamp();
    private double lastVelocity = 0;

    // Current offset of the motor (radians)
    private double offset = 0;
    
    // Conversion factor from motor rotations to mechanism units (rad or m)
    private double gearRatio = 1.0;

    private double upperLimit;
    private double lowerLimit;
    private boolean limitsEnabled=false;

    private Boolean lastInverted;
    private Boolean lastBraked;
    
    // Control modes for tracking what type of control we're using
    private enum ControlMode {
        NONE, DUTY_CYCLE, VOLTAGE, TORQUE_CURRENT, 
        POSITION_VOLTAGE, POSITION_CURRENT, POSITION_MAGIC_VOLTAGE, POSITION_MAGIC_CURRENT,
        VELOCITY_VOLTAGE, VELOCITY_CURRENT, VELOCITY_MAGIC_VOLTAGE, VELOCITY_MAGIC_CURRENT
    }

    // Make a SparkMax with the given motor type (brushed or brushless)
    public MotorIOSparkMax(int id, MotorType type) {
        motor = new SparkMax(id, type);
        encoder = motor.getEncoder();
        sim = new SparkMaxSim(motor, null);
        
        // Initialize profiled PID controller with default constraints
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10, 10); // Default max vel and accel
        profiledPidController = new ProfiledPIDController(0, 0, 0, constraints);
    }

    // Make a brushless SparkMax (most common)
    public MotorIOSparkMax(int id) {
        this(id, MotorType.kBrushless);
    }

    @Override
    public void update() {
        if (configChanged) {
            configChanged = false;
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        // Update all input values from the motor signals
        // SparkMax doesn't have isConnected(), so we'll check if we can read valid data
        try {
            double testRead = motor.getBusVoltage();
            inputs.connected = !Double.isNaN(testRead) && testRead > 0;
        } catch (Exception e) {
            inputs.connected = false;
        }

        // Convert motor rotations to mechanism units (radians or meters)
        double position = Units.rotationsToRadians(encoder.getPosition()) * gearRatio - offset;
        inputs.position = position;
        inputs.velocity = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) * gearRatio; // RPM to rad/s
        
        // Calculate acceleration from velocity change
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTime;
        if (dt > 0) {
            inputs.accel = (inputs.velocity - lastVelocity) / dt;
        }
        lastVelocity = inputs.velocity;
        lastTime = currentTime;

        inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.supplyVoltage = motor.getBusVoltage();
        inputs.supplyCurrent = motor.getOutputCurrent();
        
        // SparkMax doesn't separate torque current from supply current
        inputs.torqueCurrent = motor.getOutputCurrent();

        // Get control mode as string
        inputs.controlMode = currentControlMode.toString();

        // Run the on-RIO control loop if we're in a closed-loop mode
        runControlLoop();

        inputs.temp = motor.getMotorTemperature();
        inputs.dutyCycle = motor.getAppliedOutput();

        // Get fault information
        Faults faults = motor.getFaults();
        inputs.hardwareFault = faults.firmware || faults.sensor;
        inputs.tempFault = faults.temperature;
        inputs.forwardLimitFault = position>=upperLimit && limitsEnabled;
        inputs.reverseLimitFault = position<=lowerLimit && limitsEnabled;

        // Update alerts using the base class method (this checks all fault conditions and updates dashboard alerts)
        super.update();
    }

    // Run the on-RIO control loop and update inputs with PID component values
    private void runControlLoop() {
        double pidOutput = 0;
        double feedforwardOutput = 0;
        
        // Set setpoint and error values for logging
        inputs.setpoint = currentSetpoint;
        inputs.error = currentSetpoint - inputs.position;
        
        switch (currentControlMode) {
            case POSITION_VOLTAGE:
            case POSITION_CURRENT:
                // Regular position control
                if (continuousWrap) {
                    pidController.enableContinuousInput(-Math.PI, Math.PI);
                }
                pidOutput = pidController.calculate(inputs.position, currentSetpoint);
                feedforwardOutput = calculateFeedforward(currentSetpoint, 0); // No velocity for position
                break;
                
            case POSITION_MAGIC_VOLTAGE:
            case POSITION_MAGIC_CURRENT:
                // Motion Magic-style position control with profiling
                if (continuousWrap) {
                    profiledPidController.enableContinuousInput(-Math.PI, Math.PI);
                }
                pidOutput = profiledPidController.calculate(inputs.position, currentSetpoint);
                TrapezoidProfile.State goal = profiledPidController.getGoal();
                feedforwardOutput = calculateFeedforward(goal.position, goal.velocity);
                break;
                
            case VELOCITY_VOLTAGE:
            case VELOCITY_CURRENT:
                // Regular velocity control
                pidOutput = pidController.calculate(inputs.velocity, currentSetpoint);
                feedforwardOutput = calculateFeedforward(inputs.position, currentSetpoint);
                break;
                
            case VELOCITY_MAGIC_VOLTAGE:
            case VELOCITY_MAGIC_CURRENT:
                // Motion Magic-style velocity control (using velocity setpoint directly)
                pidOutput = pidController.calculate(inputs.velocity, currentSetpoint);
                feedforwardOutput = calculateFeedforward(inputs.position, currentSetpoint);
                break;
                
            default:
                // Open loop modes - no PID
                pidOutput = 0;
                feedforwardOutput = 0;
                break;
        }
        
        // Store PID component outputs for logging (like TalonFX does)
        inputs.propOutput = pidController.getP() * inputs.error;
        inputs.intOutput = pidController.getI() * pidController.getAccumulatedError();
        // WPILib PIDController doesn't expose previous error, so we'll approximate
        inputs.derivOutput = pidController.getD();
        inputs.feedforward = feedforwardOutput;
        
        // Apply the control output
        double totalOutput = pidOutput + feedforwardOutput;
        
        // For current modes, we'll approximate with voltage
        // This is a limitation of SparkMax - no direct current control
        motor.setVoltage(totalOutput);
        if (currentControlMode == ControlMode.POSITION_CURRENT || 
            currentControlMode == ControlMode.POSITION_MAGIC_CURRENT ||
            currentControlMode == ControlMode.VELOCITY_CURRENT ||
            currentControlMode == ControlMode.VELOCITY_MAGIC_CURRENT ||
            currentControlMode == ControlMode.TORQUE_CURRENT) {
            DriverStation.reportWarning("SparkMax doesn't support current control", false);
        }
    }
    
    // Calculate feedforward based on the gravity type and current state
    private double calculateFeedforward(double position, double velocity) {
        switch (gravityType) {
            case Arm_Cosine:
                return armFeedforward.calculate(position, velocity);
            case Elevator_Static:
                return elevatorFeedforward.calculate(velocity);
            default:
                return simpleFeedforward.calculate(velocity);
        }
    }

    // Tell the motor how fast to spin (percent, -1 = full reverse, 1 = full forward)
    @Override
    public void setSpeed(double value) {
        currentControlMode = ControlMode.DUTY_CYCLE;
        motor.set(value);
    }

    // Tell the motor what voltage to apply (volts). Similar to setSpeed but in volts.
    @Override
    public void setVoltage(double volts) {
        currentControlMode = ControlMode.VOLTAGE;
        motor.setVoltage(volts);
    }

    // Tell the motor the torque-producing current to use (amps). Use on-RIO control for better accuracy.
    @Override
    public void setTorqueCurrent(double current) {
        currentControlMode = ControlMode.TORQUE_CURRENT;
        currentSetpoint = current; // Store the current target for the control loop
        // The actual control happens in runControlLoop()
    }

    // Tell the motor to go to a target position using Motion Magic-style control with current (radians)
    @Override
    public void setGoalWithCurrentMagic(double position) {
        currentControlMode = ControlMode.POSITION_MAGIC_CURRENT;
        currentSetpoint = position;
        profiledPidController.reset(inputs.position); // Reset profiled controller to current position
    }

    // Tell the motor to go to a target position using Motion Magic-style control with voltage (radians)
    @Override
    public void setGoalWithVoltageMagic(double position) {
        currentControlMode = ControlMode.POSITION_MAGIC_VOLTAGE;
        currentSetpoint = position;
        profiledPidController.reset(inputs.position); // Reset profiled controller to current position
    }

    // Tell the motor to reach a target speed using Motion Magic-style control with current (rad/s)
    @Override
    public void setVelocityWithCurrentMagic(double velocity) {
        currentControlMode = ControlMode.VELOCITY_MAGIC_CURRENT;
        currentSetpoint = velocity;
    }

    // Tell the motor to reach a target speed using Motion Magic-style control with voltage (rad/s)
    @Override
    public void setVelocityWithVoltageMagic(double velocity) {
        currentControlMode = ControlMode.VELOCITY_MAGIC_VOLTAGE;
        currentSetpoint = velocity;
    }

    // Tell the motor to go to a target position using current control (radians)
    @Override
    public void setGoalWithCurrent(double position) {
        currentControlMode = ControlMode.POSITION_CURRENT;
        currentSetpoint = position;
    }

    // Tell the motor to go to a target position using voltage control (radians)
    @Override
    public void setGoalWithVoltage(double position) {
        currentControlMode = ControlMode.POSITION_VOLTAGE;
        currentSetpoint = position;
    }

    // Tell the motor to reach a target speed using current control (rad/s)
    @Override
    public void setVelocityWithCurrent(double velocity) {
        currentControlMode = ControlMode.VELOCITY_CURRENT;
        currentSetpoint = velocity;
    }

    // Tell the motor to reach a target speed using voltage control (rad/s)
    @Override
    public void setVelocityWithVoltage(double velocity) {
        currentControlMode = ControlMode.VELOCITY_VOLTAGE;
        currentSetpoint = velocity;
    }

    // Make this motor follow another SparkMax with the given CAN ID (invert if needed)
    @Override
    public void follow(int motorId, boolean invert) {
        // SparkMax following requires a reference to the leader motor object
        // This is a limitation - we can't follow by ID alone like TalonFX
        // When following, we disable our on-RIO control and let SparkMax handle it
        currentControlMode = ControlMode.NONE;
        config.follow(motorId, invert);
        configChanged = true;
    }

    // Tell the motor which direction is forward (true = invert)
    @Override
    public void setInverted(boolean inverted) {
        if (lastInverted==null || inverted != lastInverted) {
            config.inverted(inverted);
            configChanged = true;
            lastInverted=inverted;
        }
    }

    // Tell the motor what to do when stopped: brake (hold) or coast (freewheel)
    @Override
    public void setBraking(boolean brake) {
        if (lastBraked==null || brake!=lastBraked) {
            IdleMode newIdleMode = brake ? IdleMode.kBrake : IdleMode.kCoast;
            config.idleMode(newIdleMode);
            configChanged = true;
            lastBraked=brake;
        }
    }

    // Make PID and feedforward values active for on-RIO controllers
    @Override
    public void setkP(double kP) {
        pidController.setP(kP);
        profiledPidController.setP(kP);
    }

    @Override
    public void setkI(double kI) {
        pidController.setI(kI);
        profiledPidController.setI(kI);
    }

    @Override
    public void setkD(double kD) {
        pidController.setD(kD);
        profiledPidController.setD(kD);
    }

    // Store feedforward gains separately since WPILib feedforward controllers don't expose getters
    private double kS = 0, kG = 0, kV = 0, kA = 0;

    @Override
    public void setkS(double kS) {
        this.kS = kS;
        // Update all feedforward controllers with new kS
        simpleFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
        armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
        elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV);
    }

    @Override
    public void setkG(double kG) {
        this.kG = kG;
        // Update gravity feedforward for arm and elevator controllers
        armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
        elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV);
    }

    @Override
    public void setkV(double kV) {
        this.kV = kV;
        // Update velocity feedforward for all controllers
        simpleFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
        armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
        elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV);
    }

    @Override
    public void setkA(double kA) {
        this.kA = kA;
        // Update acceleration feedforward for controllers that support it
        simpleFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
        armFeedforward = new ArmFeedforward(kS, kG, kV, kA);
    }

    @Override
    public void setGains(Slot0Configs gains) {
        // Apply all gains to our on-RIO controllers
        setkP(gains.kP);
        setkI(gains.kI);
        setkD(gains.kD);
        setkS(gains.kS);
        setkG(gains.kG);
        setkV(gains.kV);
        setkA(gains.kA);
    }

    @Override
    public void setMaxVelocity(double maxVelocity) {
        // Update the profiled PID controller constraints for Motion Magic-style control
        TrapezoidProfile.Constraints newConstraints = new TrapezoidProfile.Constraints(
            maxVelocity, 
            profiledPidController.getConstraints().maxAcceleration
        );
        profiledPidController.setConstraints(newConstraints);
    }

    @Override
    public void setMaxAccel(double maxAccel) {
        // Update the profiled PID controller constraints for Motion Magic-style control
        TrapezoidProfile.Constraints newConstraints = new TrapezoidProfile.Constraints(
            profiledPidController.getConstraints().maxVelocity,
            maxAccel
        );
        profiledPidController.setConstraints(newConstraints);
    }

    @Override
    public void setMaxJerk(double maxJerk) {
        DriverStation.reportWarning("SparkMax doesn't support jerk limiting", false);
        // WPILib ProfiledPIDController doesn't support jerk limiting
        // This parameter will be ignored (TalonFX Motion Magic feature not available)
    }

    // Make angle wrap-around enabled (useful for swerve angles that can spin past 360°)
    @Override
    public void setContinuousWrap(boolean wrap) {
        this.continuousWrap = wrap;
        // The actual continuous input will be enabled in runControlLoop() when needed
    }

    // Tell the controller which gravity model to use (Arm_Cosine or Elevator_Static)
    @Override
    public void setFeedforwardType(GravityTypeValue type) {
        this.gravityType = type;
    }

    // Tell the motor to use a remote encoder - SparkMax doesn't support CANcoders directly
    @Override
    public void connectCANcoder(int id, double motorToSensorRatio, double sensorToMechanismRatio) {
        // SparkMax doesn't support remote CANcoders like TalonFX
        // You would need to read the CANcoder separately and use it for feedback
        // For now, we'll just store the gear ratio
        this.gearRatio = motorToSensorRatio * sensorToMechanismRatio;
    }

    // Tell the motor to use its internal sensor with a gear ratio to the mechanism (unitless)
    @Override
    public void setGearRatio(double motorToMechanismRatio) {
        this.gearRatio = motorToMechanismRatio;
        
        // We handle unit conversion in our update() method instead of relying on SparkMax conversion
        // This gives us more control and consistency with the TalonFX implementation
    }

    // Tell the motor the absolute offset of the mechanism zero (radians)
    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }

    // Limit the motor's torque-producing current (amps) - SparkMax uses smart current limit
    @Override
    public void setStatorCurrentLimit(double statorCurrentLimit) {
        config.smartCurrentLimit((int) statorCurrentLimit);
        configChanged = true;
    }

    // Limit the battery current draw (amps) - SparkMax doesn't separate supply from stator
    @Override
    public void setSupplyCurrentLimit(double supplyCurrentLimit) {
        // Use the same as stator current limit since SparkMax doesn't separate them
        setStatorCurrentLimit(supplyCurrentLimit);
    }

    @Override
    public void setSupplyCurrentLowerLimit(double supplyCurrentLowerLimit) {
        DriverStation.reportWarning("SparkMax doesn't support advanced current limiting", false);
    }

    @Override
    public void setSupplyCurrentLowerTime(double supplyCurrentLowerTime) {
        DriverStation.reportWarning("SparkMax doesn't support advanced current limiting", false);
    }

    // Set soft limits (radians)
    @Override
    public void setLimits(double min, double max) {
        // Convert from mechanism units to motor rotations
        double minRotations = Units.radiansToRotations((min + offset) / gearRatio);
        double maxRotations = Units.radiansToRotations((max + offset) / gearRatio);
        
        config.softLimit.forwardSoftLimit(maxRotations).forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(minRotations).reverseSoftLimitEnabled(true);
        configChanged = true;
    }

    // Make the simulated mechanism position update (radians)
    @Override
    public void setMechPosition(double position) {
        // Convert mechanism position to motor rotations
        double motorRotations = Units.radiansToRotations((position + offset) / gearRatio);
        sim.setPosition(motorRotations);
    }

    // Make the simulated mechanism velocity update (rad/s)
    @Override
    public void setMechVelocity(double velocity) {
        // Convert mechanism velocity to motor RPM
        double motorRpm = Units.radiansPerSecondToRotationsPerMinute(velocity / gearRatio);
        sim.setVelocity(motorRpm);
    }

    // Clear all sticky faults on this motor
    @Override
    public void clearStickyFaults() {
        motor.clearFaults();
    }
}