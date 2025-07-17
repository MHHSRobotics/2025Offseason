package frc.robot.io;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import org.littletonrobotics.junction.AutoLog;

// See TalonFXIO for an explanation
public class CANcoderIO {
    @AutoLog
    public static class CANcoderIOInputs {
        public boolean connected;

        public double positionRad; // mechanism radians
        public double velocityRadPerSec; // mechanism radians per sec

        // Encoder faults that show up in AdvantageScope/Elastic
        public boolean badMagnetFault;
        public boolean hardwareFault;
        public boolean bootDuringEnable;
        public boolean undervoltage;
    }

    // Gets data from sensors and updates the inputs
    public void updateInputs(CANcoderIOInputs inputs) {}

    // Applies a config to the encoder
    public void applyConfig(CANcoderConfiguration config) {}

    // CTRE doesn't let us set the gear ratio through the config so we have to do it manually
    public void setEncoderRatio(double encoderRatio) {}
}
