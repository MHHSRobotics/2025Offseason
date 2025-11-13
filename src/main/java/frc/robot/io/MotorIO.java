package frc.robot.io;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

// Make a simple motor interface used by subsystems (arms, elevators, flywheels).
// All position/velocity/acceleration values are doubles in mechanism units (radians for rotary, meters for linear).
// - For rotary mechanisms (arms, flywheels): units are radians, rad/s, rad/s^2
// - For linear mechanisms (elevators): configure the gear ratio to include spool geometry conversion
//   (e.g., 0.05m spool diameter: gearRatio = mechanicalRatio * (1 / 0.05) to treat meters as radians)
public class MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public boolean connected; // Is the motor controller connected

        public double positionRad; // Mechanism position (rad or m)
        public double velocityRadPerSec; // Mechanism velocity (rad/s or m/s)
        public double accelRadPerSecSquared; // Mechanism acceleration (rad/s^2 or m/s^2)

        public double appliedVoltageVolts; // Applied voltage (volts)
        public double supplyVoltageVolts; // Battery voltage (volts)
        public double supplyCurrentAmps; // Battery current draw (amps)
        public double torqueCurrentAmps; // Motor torque proxy (amps)

        public String controlMode; // Current control type (e.g., DutyCycle, Voltage, MotionMagic)

        public double setpoint; // Current target position (rad or m)
        public double setpointVelocity; // Target velocity (rad/s or m/s)
        public double error; // Target minus position (rad or m)
        public double feedforward; // Controller feedforward (often amps for current control)
        // Controller outputs (units depend on control mode: volts for voltage control, amps for current control)
        public double derivOutput; // kD contribution
        public double intOutput; // kI contribution
        public double propOutput; // kP contribution

        public double tempCelsius; // Controller temperature (C)

        public double dutyCycle; // Duty cycle command (-1 to 1)

        // Difference between encoder and motor readings (rad or m)
        public double encoderDiffRad;

        // Fault flags
        public boolean hardwareFault;
        public boolean tempFault;
        public boolean forwardLimitFault;
        public boolean reverseLimitFault;

        // Rotor position w/o encoder fusion (rad)
        public double rotorPositionRad;
    }

    private String logPath;

    private String name;

    // Alert objects to show motor problems on the dashboard
    private Alert disconnectAlert;
    private Alert hardwareFaultAlert;
    private Alert tempFaultAlert;
    private Alert forwardLimitAlert;
    private Alert reverseLimitAlert;

    public MotorIO(String name, String logPath) {
        this.name = name;
        this.logPath = logPath;

        // Create alerts with descriptive names for this motor
        disconnectAlert = new Alert("The " + name + " is disconnected", AlertType.kError);
        hardwareFaultAlert = new Alert("The " + name + " encountered an internal hardware fault", AlertType.kError);
        tempFaultAlert = new Alert("The " + name + " is overheating!", AlertType.kWarning);
        forwardLimitAlert = new Alert("The " + name + " hit its forward limit", AlertType.kWarning);
        reverseLimitAlert = new Alert("The " + name + " hit its reverse limit", AlertType.kWarning);
    }

    public String getName() {
        return name;
    }

    protected MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    // Find out the latest values from the motor and store them in inputs
    public void update() {
        // Log the inputs to AdvantageKit
        Logger.processInputs(logPath, inputs);

        // Update alerts based on the current motor status (this runs after subclass updates inputs)
        disconnectAlert.set(!inputs.connected);
        hardwareFaultAlert.set(inputs.hardwareFault);
        tempFaultAlert.set(inputs.tempFault);
        forwardLimitAlert.set(inputs.forwardLimitFault);
        reverseLimitAlert.set(inputs.reverseLimitFault);
    }

    private void unsupportedFeature() {
        if (Constants.currentMode != Mode.REPLAY) {
            Alerts.create("An unsupported feature was used on " + getName(), AlertType.kWarning);
        }
    }

    // Find out the current inputs snapshot (read-only)
    public MotorIOInputs getInputs() {
        return inputs;
    }

    // Tell the motor how fast to spin (percent, -1 = full reverse, 1 = full forward)
    public void setDutyCycle(double value) {
        unsupportedFeature();
    }

    // Tell the motor what voltage to apply (volts)
    public void setVoltage(double volts) {
        unsupportedFeature();
    }

    // Tell the motor the torque-producing current to use (amps)
    public void setTorqueCurrent(double amps) {
        unsupportedFeature();
    }

    // Tell the motor to go to a target position using Motion Magic with current control (mechanism units)
    public void setGoalWithCurrentMagic(double goal, Supplier<Double> feedforward) {
        unsupportedFeature();
    }

    public void setGoalWithCurrentMagic(double goal) {
        setGoalWithCurrentMagic(goal, null);
    }

    // Tell the motor to go to a target position using Motion Magic with voltage control (mechanism units)
    public void setGoalWithVoltageMagic(double goal, Supplier<Double> feedforward) {
        unsupportedFeature();
    }

    public void setGoalWithVoltageMagic(double goal) {
        setGoalWithVoltageMagic(goal, null);
    }

    // Tell the motor to reach a target speed using Motion Magic with current control (mechanism units/s)
    public void setVelocityWithCurrentMagic(double velocity, Supplier<Double> feedforward) {
        unsupportedFeature();
    }

    public void setVelocityWithCurrentMagic(double velocity) {
        setVelocityWithCurrentMagic(velocity, null);
    }

    // Tell the motor to reach a target speed using Motion Magic with voltage control (mechanism units/s)
    public void setVelocityWithVoltageMagic(double velocity, Supplier<Double> feedforward) {
        unsupportedFeature();
    }

    public void setVelocityWithVoltageMagic(double velocity) {
        setVelocityWithVoltageMagic(velocity, null);
    }

    // Tell the motor to go to a target position using current control (mechanism units)
    public void setGoalWithCurrent(double goal, Supplier<Double> feedforward) {
        unsupportedFeature();
    }

    public void setGoalWithCurrent(double goal) {
        setGoalWithCurrent(goal, null);
    }

    // Tell the motor to go to a target position using voltage control (mechanism units)
    public void setGoalWithVoltage(double goal, Supplier<Double> feedforward) {
        unsupportedFeature();
    }

    public void setGoalWithVoltage(double goal) {
        setGoalWithVoltage(goal, null);
    }

    // Tell the motor to reach a target speed using current control (mechanism units/s)
    public void setVelocityWithCurrent(double velocity, Supplier<Double> feedforward) {
        unsupportedFeature();
    }

    public void setVelocityWithCurrent(double velocity) {
        setVelocityWithCurrent(velocity, null);
    }

    // Tell the motor to reach a target speed using voltage control (mechanism units/s)
    public void setVelocityWithVoltage(double velocity, Supplier<Double> feedforward) {
        unsupportedFeature();
    }

    public void setVelocityWithVoltage(double velocity) {
        setVelocityWithVoltage(velocity, null);
    }

    // Make this motor follow another motor, invert if needed
    public void follow(MotorIO motor, boolean invert) {
        unsupportedFeature();
    }

    // Tell the motor which direction is forward (true = invert)
    public void setInverted(boolean inverted) {
        unsupportedFeature();
    }

    // Tell the motor what to do when stopped: brake (hold) or coast (freewheel)
    public void setBraking(boolean braking) {
        unsupportedFeature();
    }

    // Make the proportional gain (kP) value active
    public void setkP(double kP) {
        unsupportedFeature();
    }

    // Make the derivative gain (kD) value active
    public void setkD(double kD) {
        unsupportedFeature();
    }

    // Make the integral gain (kI) value active
    public void setkI(double kI) {
        unsupportedFeature();
    }

    // Make the gravity feedforward (kG) value active (helps hold arms/elevators)
    public void setkG(double kG) {
        unsupportedFeature();
    }

    // Make the static feedforward (kS) value active (helps start motion)
    public void setkS(double kS) {
        unsupportedFeature();
    }

    // Make the velocity feedforward (kV) value active (scales with speed)
    public void setkV(double kV) {
        unsupportedFeature();
    }

    // Make the acceleration feedforward (kA) value active (helps with quick moves)
    public void setkA(double kA) {
        unsupportedFeature();
    }

    // Make a full Slot0 gains config active
    public void setGains(Slot0Configs gains) {
        unsupportedFeature();
    }

    // Tell Motion Magic the max speed to use (mechanism units/s)
    public void setMaxVelocity(double maxVelocity) {
        unsupportedFeature();
    }

    // Tell Motion Magic the max acceleration to use (mechanism units/s^2)
    public void setMaxAccel(double maxAccel) {
        unsupportedFeature();
    }

    // Tell Motion Magic the max jerk to use (mechanism units/s^3)
    public void setMaxJerk(double maxJerk) {
        unsupportedFeature();
    }

    // Make angle wrap-around enabled (useful for swerve angles that can spin past 360°)
    public void setContinuousWrap(boolean wrap) {
        unsupportedFeature();
    }

    // Tell the controller which gravity model to use (like Arm_Cosine or Elevator_Static)
    public void setFeedforwardType(GravityTypeValue type) {
        unsupportedFeature();
    }

    // Whether to use closed loop sign (sign of difference between setpoint and position) or velocity sign (sign of angular velocity) for kS
    public void setStaticFeedforwardSign(StaticFeedforwardSignValue v) {
        unsupportedFeature();
    }

    // Tell the motor to use a remote encoder with given gear ratio (unitless). Make sure to set encoder settings before
    // calling this.
    public void connectEncoder(EncoderIO encoder, double motorToSensorRatio, boolean fuse) {
        unsupportedFeature();
    }

    // Fuse defaults to true. Make sure to set encoder settings before calling this.
    public void connectEncoder(EncoderIO encoder, double motorToSensorRatio) {
        connectEncoder(encoder, motorToSensorRatio, true);
    }

    // Tell the motor to use its internal sensor with a gear ratio to the mechanism (unitless). Don't call both this and
    // connectEncoder.
    public void setGearRatio(double gearRatio) {
        unsupportedFeature();
    }

    // Tell the motor the absolute offset of the mechanism zero (mechanism units). Do this AFTER connecting the encoder and the GravityType.
    public void setOffset(double offset) {
        unsupportedFeature();
    }

    // Limit the motor's torque-producing current (amps)
    public void setStatorCurrentLimit(double amps) {
        unsupportedFeature();
    }

    // Limit the battery current draw (amps)
    public void setSupplyCurrentLimit(double amps) {
        unsupportedFeature();
    }

    // Lower the current limit to this amount (amps) after a brownout condition
    public void setSupplyCurrentLowerLimit(double amps) {
        unsupportedFeature();
    }

    // Time (seconds) above the limit before lowering the current
    public void setSupplyCurrentLowerTime(double seconds) {
        unsupportedFeature();
    }

    // Set soft limits (mechanism units). Do this AFTER setting the offset.
    public void setLimits(double min, double max) {
        unsupportedFeature();
    }

    // Clear all sticky faults on this motor
    public void clearStickyFaults() {
        unsupportedFeature();
    }

    // Tell the motor to be disabled or enabled
    public void setDisabled(boolean disabled) {
        unsupportedFeature();
    }

    // Make the simulated mechanism position update (mechanism units). Simulation-only.
    public void setMechPosition(double position) {
        unsupportedFeature();
    }

    // Make the simulated mechanism velocity update (mechanism units/s). Simulation-only.
    public void setMechVelocity(double velocity) {
        unsupportedFeature();
    }

    // Set whether the simulated motor is connected. Simulation-only.
    public void setConnected(boolean connected) {
        unsupportedFeature();
    }
}
