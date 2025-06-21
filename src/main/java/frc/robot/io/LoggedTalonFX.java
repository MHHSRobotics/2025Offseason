package frc.robot.io;

import org.littletonrobotics.junction.Logger;

public class LoggedTalonFX {
    private String logPath;
    private final TalonFXIO io;
    private final TalonFXIOInputsAutoLogged inputs = new TalonFXIOInputsAutoLogged();

    public LoggedTalonFX(TalonFXIO io, String logPath) {
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

    public double getGoal() {
        return inputs.setpoint;
    }

    public void setSpeed(double value) {
        io.setSpeed(value);
    }

    public void setGoal(double position) {
        io.setGoal(position);
    }
}
