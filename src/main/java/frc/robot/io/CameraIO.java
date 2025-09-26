package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Alerts;

public class CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected;

        // Camera pose data as separate arrays (AutoLog doesn't support complex types like List<Pair<Pose2d, Double>>)
        public double[] poseXMeters = new double[64]; // X position of each pose (meters)
        public double[] poseYMeters = new double[64]; // Y position of each pose (meters)
        public double[] poseRotationRad = new double[64]; // Rotation of each pose (radians)
        public double[] poseTimestamps = new double[64]; // Timestamp of each pose (seconds)

        public int measurements;
    }

    private String name;
    private String logPath;

    private Alert disconnectAlert;

    public CameraIO(String name, String logPath) {
        this.name = name;
        this.logPath = logPath;

        // Create alerts with descriptive names for this camera
        disconnectAlert = new Alert("The " + name + " is disconnected", AlertType.kError);
    }

    public String getName() {
        return name;
    }

    protected CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    public void update() {
        // Log the inputs to AdvantageKit
        Logger.processInputs(logPath, inputs);

        // Update alerts based on the current encoder status (this runs after subclass updates inputs)
        disconnectAlert.set(!inputs.connected);
    }

    public CameraIOInputs getInputs() {
        return inputs;
    }

    private void unsupportedFeature() {
        if (Constants.currentMode != Mode.REPLAY) {
            Alerts.create("An unsupported feature was used on " + getName(), AlertType.kWarning);
        }
    }

    public void clearMeasurements() {
        unsupportedFeature();
    }
}
