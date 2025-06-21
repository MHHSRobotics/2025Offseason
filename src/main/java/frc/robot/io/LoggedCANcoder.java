package frc.robot.io;

import org.littletonrobotics.junction.Logger;

public class LoggedCANcoder {
    private String logPath;
    private final CANcoderIO io;
    private final CANcoderIOInputsAutoLogged inputs = new CANcoderIOInputsAutoLogged();

    public LoggedCANcoder(CANcoderIO io, String logPath) {
        this.io = io;
        this.logPath = logPath;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocity;
    }
}
