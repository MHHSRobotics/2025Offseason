package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

public class GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected;
        public double yawPositionRad;
        public double yawVelocityRadPerSec;
        public boolean hardwareFault;
    }

    protected GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public void updateInputs() {}

    public GyroIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public void setMechYaw(double yaw) {}

    public void setMechYawVelocity(double yawVelocity) {}
}
