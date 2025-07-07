package frc.robot.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import org.littletonrobotics.junction.AutoLog;

// This is a generic class for all IO objects that access a TalonFX. The abstraction allows us to use different
// subclasses depending on if we're in the real bot or simulation, so we don't have to worry about simulation in the
// main robot code.
public class TalonFXIO {

    // Creates a TalonFXIOInputsAutoLogged class that logs all inputs every periodic(). That constructor should be used
    // instead of the default one here.
    @AutoLog
    // Inputs class for TalonFXIO. This is instantiated and stored whenever a TalonFXIO is. It ensures that all inputs
    // to the robot code from external sensors are processed by AdvantageKit to make replay possible.
    public static class TalonFXIOInputs {
        public boolean connected;

        // Mechanism units are radians for arms and flywheels and meters for elevators.
        public double position; // mechanism units
        public double velocity; // mechanism units per sec
        public double accel; // mechanism units per sec^2

        public double appliedVoltage; // volts
        public double supplyVoltage; // volts
        public double supplyCurrent; // Battery current draw, amps
        public double torqueCurrent; // Motor torque indicator, amps

        public String controlMode; // Current control type

        public double setpoint; // Current setpoint for the motor, mechanism units
        public double error; // Difference between current position and goal position, mechanism units
        public double feedforward; // Current feedforward for the motor, amps
        public double derivOutput; // Current motor output from kD, amps
        public double intOutput; // Current motor output from kI, amps
        public double propOutput; // Current motor output from kP, amps

        public double temp; // Temperature, celsius

        public double dutyCycle; // Current duty cycle for the motor, -1 to 1

        // Motor faults that show up as alerts in AdvantageScope/Elastic
        public boolean hardwareFault;
        public boolean procTempFault;
        public boolean deviceTempFault;
        public boolean undervoltageFault;
        public boolean bootDuringEnable;
        public boolean forwardHardLimit;
        public boolean forwardSoftLimit;
        public boolean reverseHardLimit;
        public boolean reverseSoftLimit;
    }

    // Gets data from external sensors and updates the TalonFXIOInputs object
    public void updateInputs(TalonFXIOInputs inputs) {}

    // Applies a config to the internal TalonFX
    public void applyConfig(TalonFXConfiguration config) {}

    // Sets the duty cycle of the motor
    public void setControl(DutyCycleOut control) {}

    // Sets the voltage output of the motor
    public void setControl(VoltageOut control) {}

    // Uses MotionMagic to target a position, with torque current
    public void setControl(MotionMagicTorqueCurrentFOC control) {}

    // Uses MotionMagic to target a position, with voltage
    public void setControl(MotionMagicVoltage control) {}
}
