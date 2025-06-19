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

    private CANcoderIOInputsAutoLogged inputs = new CANcoderIOInputsAutoLogged();

    public void updateInputs() {}

    public void applyConfig(CANcoderConfiguration config) {}

    public CANcoderIOInputsAutoLogged getInputs() {
        return inputs;
    }
}
