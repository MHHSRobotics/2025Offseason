package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

// Make a simple motor interface used by subsystems (arms, elevators, flywheels).
// Mechanism units:
// - Arms/flywheels use radians (rad) and radians per second (rad/s)
// - Elevators use meters (m) and meters per second (m/s)
public class MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public boolean connected; // Is the motor controller connected

        // Mechanism values (see units above)
        public double position; // mechanism position (rad or m)
        public double velocity; // mechanism speed (rad/s or m/s)
        public double accel; // mechanism acceleration (rad/s^2 or m/s^2)

        public double appliedVoltage; // Applied voltage (volts)
        public double supplyVoltage; // Battery voltage (volts)
        public double supplyCurrent; // Battery current draw (amps)
        public double torqueCurrent; // Motor torque proxy (amps)

        public String controlMode; // Current control type (e.g., DutyCycle, Voltage, MotionMagic)

        public double setpoint; // Current target (mechanism units)
        public double error; // Target minus position (mechanism units)
        public double feedforward; // Controller feedforward (often amps for current control)
        public double derivOutput; // kD contribution (controller units)
        public double intOutput; // kI contribution (controller units)
        public double propOutput; // kP contribution (controller units)

        public double temp; // Controller temperature (C)

        public double dutyCycle; // Duty cycle command (-1 to 1)

        public boolean hardwareFault;
        public boolean tempFault;
        public boolean forwardLimitFault;
        public boolean reverseLimitFault;
    }

    private String logPath = "";

    // Alert objects to show motor problems on the dashboard
    private Alert disconnectAlert;
    private Alert hardwareFaultAlert;
    private Alert tempFaultAlert;
    private Alert forwardLimitAlert;
    private Alert reverseLimitAlert;

    public MotorIO() {
        // Alerts will be created when setName() is called
    }

    // Tell the MotorIO what to call this motor for alerts (like "arm" or "FL drive")
    public void setName(String name) {
        // Create alerts with descriptive names for this motor
        disconnectAlert = new Alert("The " + name + " motor is disconnected", AlertType.kError);
        hardwareFaultAlert =
                new Alert("The " + name + " motor encountered an internal hardware fault", AlertType.kError);
        tempFaultAlert = new Alert("The " + name + " motor is overheating!", AlertType.kWarning);
        forwardLimitAlert = new Alert("The " + name + " motor hit its forward limit", AlertType.kWarning);
        reverseLimitAlert = new Alert("The " + name + " motor hit its reverse limit", AlertType.kWarning);
    }

    // Tell the MotorIO where to log its data (like "Arm/Motor" or "Drive/Module0/DriveMotor")
    public void setPath(String path) {
        this.logPath = path;
    }

    protected MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    // Find out the latest values from the motor and store them in inputs
    public void update() {
        // Log the inputs to AdvantageKit if a path has been set
        if (!logPath.isEmpty()) {
            Logger.processInputs(logPath, inputs);
        }

        // Update alerts based on the current motor status (this runs after subclass updates inputs)
        // Only update alerts if they've been created (setName() was called)
        if (disconnectAlert != null) {
            disconnectAlert.set(!inputs.connected);
            hardwareFaultAlert.set(inputs.hardwareFault);
            tempFaultAlert.set(inputs.tempFault);
            forwardLimitAlert.set(inputs.forwardLimitFault);
            reverseLimitAlert.set(inputs.reverseLimitFault);
        }
    }

    // Find out the current inputs snapshot (read-only)
    public MotorIOInputsAutoLogged getInputs() {
        return inputs;
    }

    // Tell the motor how fast to spin (percent, -1 = full reverse, 1 = full forward)
    public void setSpeed(double value) {}

    // Tell the motor what voltage to apply (volts). Similar to setSpeed but in volts.
    public void setVoltage(double volts) {}

    // Tell the motor the torque-producing current to use (amps). Helpful to ignore battery sag.
    public void setTorqueCurrent(double current) {}

    // Tell the motor to go to a target position using Motion Magic with current control (mechanism units)
    public void setGoalWithCurrentMagic(double goal) {}

    // Tell the motor to go to a target position using Motion Magic with voltage control (mechanism units)
    public void setGoalWithVoltageMagic(double goal) {}

    // Tell the motor to reach a target speed using Motion Magic with current control (mechanism units per second)
    public void setVelocityWithCurrentMagic(double velocity) {}

    // Tell the motor to reach a target speed using Motion Magic with voltage control (mechanism units per second)
    public void setVelocityWithVoltageMagic(double velocity) {}

    // Tell the motor to go to a target position using current control (mechanism units)
    public void setGoalWithCurrent(double goal) {}

    // Tell the motor to go to a target position using voltage control (mechanism units)
    public void setGoalWithVoltage(double goal) {}

    // Tell the motor to reach a target speed using current control (mechanism units per second)
    public void setVelocityWithCurrent(double velocity) {}

    // Tell the motor to reach a target speed using voltage control (mechanism units per second)
    public void setVelocityWithVoltage(double velocity) {}

    // Make this motor follow another motor with the given CAN ID (invert if needed)
    public void follow(int motorId, boolean invert) {}

    // Tell the motor which direction is forward (true = invert)
    public void setInverted(boolean inverted) {}

    // Tell the motor what to do when stopped: brake (hold) or coast (freewheel)
    public void setBraking(boolean braking) {}

    // Make the proportional gain (kP) value active
    public void setkP(double kP) {}

    // Make the derivative gain (kD) value active
    public void setkD(double kD) {}

    // Make the integral gain (kI) value active
    public void setkI(double kI) {}

    // Make the gravity feedforward (kG) value active (helps hold arms/elevators)
    public void setkG(double kG) {}

    // Make the static feedforward (kS) value active (helps start motion)
    public void setkS(double kS) {}

    // Make the velocity feedforward (kV) value active (scales with speed)
    public void setkV(double kV) {}

    // Make the acceleration feedforward (kA) value active (helps with quick moves)
    public void setkA(double kA) {}

    // Make a full Slot0 gains config active
    public void setGains(Slot0Configs gainss) {}

    // Tell Motion Magic the max speed to use (mechanism units per second)
    public void setMaxVelocity(double maxVelocity) {}

    // Tell Motion Magic the max acceleration to use (mechanism units per second^2)
    public void setMaxAccel(double maxAccel) {}

    // Tell Motion Magic the max jerk to use (mechanism units per second^3)s
    public void setMaxJerk(double maxJerk) {}

    // Make angle wrap-around enabled (useful for swerve angles that can spin past 360°)
    public void setContinuousWrap(boolean wrap) {}

    // Tell the controller which gravity model to use (like Arm_Cosine or Elevator_Static)
    public void setFeedforwardType(GravityTypeValue type) {}

    // Tell the motor to use a remote CANcoder (id) with given gear ratios (unitless)
    public void connectCANcoder(int id, double motorToSensorRatio, double sensorToMechanismRatio) {}

    // Tell the motor to use its internal sensor with a gear ratio to the mechanism (unitless)
    public void setGearRatio(double gearRatio) {}

    // Tell the motor the absolute offset of the mechanism zero (radians)
    public void setOffset(double offset) {}

    // Limit the motor's torque-producing current (amps)
    public void setStatorCurrentLimit(double statorCurrentLimit) {}

    // Limit the battery current draw (amps)
    public void setSupplyCurrentLimit(double supplyCurrentLimit) {}

    // Lower the current limit to this amount (amps) after a brownout condition
    public void setSupplyCurrentLowerLimit(double supplyCurrentLowerLimit) {}

    // Time (seconds) above the limit before lowering the current
    public void setSupplyCurrentLowerTime(double supplyCurrentLowerTime) {}

    // Set soft limits (radians)
    public void setLimits(double min, double max) {}

    // Make the simulated mechanism position update (radians)
    public void setMechPosition(double position) {}

    // Make the simulated mechanism velocity update (rad/s)
    public void setMechVelocity(double velocity) {}

    // Clear all sticky faults on this motor
    public void clearStickyFaults() {}
}
