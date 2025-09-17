package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class EncoderIO {
    @AutoLog
    public static class EncoderIOInputs {
        public boolean connected;

        public double positionRad; // mechanism radians
        public double velocityRadPerSec; // mechanism radians per sec

        public boolean badMagnetFault;
        public boolean hardwareFault;
    }

    private String logPath = "";

    private String name;

    // Alert objects to show encoder problems on the dashboard
    private Alert disconnectAlert;
    private Alert hardwareFaultAlert;
    private Alert magnetFaultAlert;

    public EncoderIO() {
        // Alerts will be created when setName() is called
    }

    // Tell the EncoderIO what to call this encoder for alerts (like "arm encoder" or "FL encoder")
    public void setName(String name) {
        this.name = name;

        // Create alerts with descriptive names for this encoder
        disconnectAlert = new Alert("The " + name + " encoder is disconnected", AlertType.kError);
        hardwareFaultAlert =
                new Alert("The " + name + " encoder encountered an internal hardware fault", AlertType.kError);
        magnetFaultAlert = new Alert("The " + name + " encoder magnet is not functioning", AlertType.kError);
    }

    public String getName() {
        return name == null ? "encoder" : name;
    }

    // Tell the EncoderIO where to log its data (like "Arm/Encoder" or "Drive/Module0/AngleEncoder")
    public void setPath(String path) {
        this.logPath = path;
    }

    protected EncoderIOInputsAutoLogged inputs = new EncoderIOInputsAutoLogged();

    // Find out the latest values from the encoder and store them in inputs
    public void update() {
        // Log the inputs to AdvantageKit if a path has been set
        if (!logPath.isEmpty()) {
            Logger.processInputs(logPath, inputs);
        }

        // Update alerts based on the current encoder status (this runs after subclass updates inputs)
        // Only update alerts if they've been created (setName() was called)
        if (disconnectAlert != null) {
            disconnectAlert.set(!inputs.connected);
            hardwareFaultAlert.set(inputs.hardwareFault);
            magnetFaultAlert.set(inputs.badMagnetFault);
        }
    }

    public EncoderIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public void setGearRatio(double ratio) {}

    public void setOffset(double offset) {}

    public void setInverted(boolean inverted) {}

    public void setMechPosition(double position) {}

    public void setMechVelocity(double velocity) {}
}
