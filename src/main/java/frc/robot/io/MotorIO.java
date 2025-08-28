package frc.robot.io;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import org.littletonrobotics.junction.AutoLog;

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

    protected MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    // Find out the latest values from the motor and store them in inputs
    public void updateInputs() {}

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

    // Set the forward soft limit (radians)
    public void setForwardLimit(double forwardLimit) {}

    // Set the reverse soft limit (radians)
    public void setReverseLimit(double reverseLimit) {}

    // Make the simulated mechanism position update (radians)
    public void setMechPosition(double position) {}

    // Make the simulated mechanism velocity update (rad/s)
    public void setMechVelocity(double velocity) {}
}
