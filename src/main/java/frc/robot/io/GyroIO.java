package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

public class GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected;
        public double yawPositionRad;
        public double yawVelocityRadPerSec;
        public boolean hardwareFault;
    }

    private String logPath = "";

    private String name;

    // Alert objects to show gyro problems on the dashboard
    private Alert disconnectAlert;
    private Alert hardwareFaultAlert;

    public GyroIO() {
        // Alerts will be created when setName() is called
    }

    // Tell the GyroIO what to call this gyro for alerts (like "gyro" or "main gyro")
    public void setName(String name) {
        this.name = name;

        // Create alerts with descriptive names for this gyro
        disconnectAlert = new Alert("The " + name + " is disconnected", AlertType.kError);
        hardwareFaultAlert = new Alert("The " + name + " has encountered a hardware fault", AlertType.kError);
    }

    public String getName() {
        return name == null ? "gyro" : name;
    }

    // Tell the GyroIO where to log its data (like "Drive/Gyro")
    public void setPath(String path) {
        this.logPath = path;
    }

    protected GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    // Find out the latest values from the gyro and store them in inputs
    public void update() {
        // Log the inputs to AdvantageKit if a path has been set
        if (!logPath.isEmpty()) {
            Logger.processInputs(logPath, inputs);
        }

        // Update alerts based on the current gyro status (this runs after subclass updates inputs)
        // Only update alerts if they've been created (setName() was called)
        if (disconnectAlert != null) {
            disconnectAlert.set(!inputs.connected);
            hardwareFaultAlert.set(inputs.hardwareFault);
        }
    }

    public GyroIOInputsAutoLogged getInputs() {
        return inputs;
    }

    private void unsupportedFeature() {
        if (Constants.currentMode != Mode.REPLAY) {
            Alerts.create("An unsupported feature was used on " + getName(), AlertType.kWarning);
        }
    }

    public void setMechYaw(double yaw) {
        unsupportedFeature();
    }

    public void setMechYawVelocity(double yawVelocity) {
        unsupportedFeature();
    }

    public void disconnect() {
        unsupportedFeature();
    }
}
