package frc.robot.io;

public class Gyro {
    private GyroIO io;
    private GyroIOInputsAutoLogged inputs;
    private String logPath;

    public Gyro(String logPath, GyroIO io) {
        this.io = io;
        this.logPath = logPath;
        inputs = new GyroIOInputsAutoLogged();
    }
}
