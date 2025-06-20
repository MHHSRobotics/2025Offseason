package frc.robot.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import org.littletonrobotics.junction.AutoLog;

public class CANcoderIO {
    @AutoLog
    public static class CANcoderIOInputs {
        public boolean connected;
        public double position; // radians
        public double velocity; // radians per sec
    }

    public void updateInputs(CANcoderIOInputs inputs) {}

    public void applyConfig(CANcoderConfiguration config) {}
}
