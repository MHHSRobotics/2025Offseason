package frc.robot.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

import org.littletonrobotics.junction.AutoLog;

public class TalonFXIO {
    @AutoLog
    public static class TalonFXIOInputs {
        public boolean connected;

        public double positionRad; // radians of mechanism
        public double velocityRadPerSec; // radians of mechanism per sec
        public double accelRadPerSecSquared; // radians of mechanism per sec^2

        public double appliedVoltage; // volts
        public double supplyVoltage; // volts
        public double supplyCurrent; // Battery current draw, amps
        public double torqueCurrent; // Motor torque indicator, amps

        public String controlMode; // Current control type

        public double setpointRad; // Current setpoint for the motor, radians of mechanism
        public double errorRad; // Difference between current position and goal position, radians of mechanism
        public double feedforward; // Current feedforward for the motor, amps
        public double derivOutput; // Current motor output from kD, amps
        public double intOutput; // Current motor output from kI, amps
        public double propOutput; // Current motor output from kP, amps

        public double temp; // Temperature, celsius

        public double dutyCycle; // Current duty cycle for the motor, -1 to 1

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

    public void updateInputs(TalonFXIOInputs inputs) {}

    public void applyConfig(TalonFXConfiguration config) {}

    public void setControl(DutyCycleOut control) {}

    public void setControl(MotionMagicTorqueCurrentFOC control) {}
}
