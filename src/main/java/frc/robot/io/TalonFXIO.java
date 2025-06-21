package frc.robot.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import org.littletonrobotics.junction.AutoLog;

public class TalonFXIO {
    @AutoLog
    public static class TalonFXIOInputs {
        public boolean connected;
        public double position; // radians of mechanism
        public double velocity; // radians of mechanism per sec
        public double appliedVoltage;
        public double supplyVoltage;
        public double supplyCurrent; // Battery current draw
        public double statorCurrent; // Motor torque indicator
        public double temp;
        public double setpoint; // Command being sent
        public String controlMode; // Control type for debugging
        public double error;
    }

    public void updateInputs(TalonFXIOInputs inputs) {}

    public void applyConfig(TalonFXConfiguration config) {}

    public void setSpeed(double value) {}

    public void setGoal(double pos) {}
}
