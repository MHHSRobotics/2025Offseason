package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

public class EncoderIO {
    @AutoLog
    public static class EncoderIOInputs {
        public boolean connected;

        public double positionRad; // mechanism radians
        public double velocityRadPerSec; // mechanism radians per sec

        public boolean badMagnetFault;
        public boolean hardwareFault;
    }

    protected EncoderIOInputsAutoLogged inputs = new EncoderIOInputsAutoLogged();

    public void updateInputs() {}

    public EncoderIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public void setRatio(double ratio) {}

    public void setInverted(boolean inverted) {}

    public void setMechPosition(double position) {}

    public void setMechVelocity(double velocity) {}
}
