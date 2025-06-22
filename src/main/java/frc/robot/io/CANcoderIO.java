package frc.robot.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import org.littletonrobotics.junction.AutoLog;

public class CANcoderIO {
    @AutoLog
    public static class CANcoderIOInputs {
        public boolean connected;

        public double positionRad; // mechanism radians
        public double velocityRadPerSec; // mechanism radians per sec

        public boolean badMagnetFault;
        public boolean hardwareFault;
        public boolean bootDuringEnable;
        public boolean undervoltage;
    }

    public void updateInputs(CANcoderIOInputs inputs) {}

    public void applyConfig(CANcoderConfiguration config) {}
}
