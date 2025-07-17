package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

    // Alerts for faults. These will appear on AdvantageScope/Elastic
    private Alert disconnectedAlert;

    private Alert badMagnetAlert;
    private Alert hardwareAlert;
    private Alert bootDuringEnableAlert;
    private Alert undervoltageAlert;

    // Current CANcoder config
    private CANcoderConfiguration config;

    public LoggedCANcoder(CANcoderIO io, String logPath, CANcoderConfiguration config) {
        this.io = io;
        this.logPath = logPath;
        this.config = config;

        // Initialize alerts
        disconnectedAlert = new Alert(logPath + " is disconnected", AlertType.kError);

        badMagnetAlert = new Alert(logPath + " has a bad magnet", AlertType.kWarning);
        hardwareAlert = new Alert(logPath + " encountered a hardware fault", AlertType.kWarning);
        bootDuringEnableAlert = new Alert(logPath + " booted during enable", AlertType.kWarning);
        undervoltageAlert = new Alert(logPath + " has insufficient voltage", AlertType.kWarning);

        // Apply the config
        updateConfig();
    }

    public void periodic() {
        // Get new inputs from CANcoderIO and log them
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);

        // Update alert status
        disconnectedAlert.set(!inputs.connected);

        badMagnetAlert.set(inputs.badMagnetFault);
        hardwareAlert.set(inputs.hardwareFault);
        bootDuringEnableAlert.set(inputs.bootDuringEnable);
        undervoltageAlert.set(inputs.undervoltage);
    }

    public double getPosition() {
        return inputs.positionRad;
    }

    public double getVelocity() {
        return inputs.velocityRadPerSec;
    }

    public CANcoderIOInputs getInputs() {
        return inputs;
    }

    private void updateConfig() {
        io.applyConfig(config);
    }
}
