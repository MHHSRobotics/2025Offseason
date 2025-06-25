package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.CANcoderIO.CANcoderIOInputs;

public class LoggedCANcoder {
    private String logPath;

    private final CANcoderIO io;
    private final CANcoderIOInputsAutoLogged inputs = new CANcoderIOInputsAutoLogged();

    private Alert disconnectedAlert;

    private Alert badMagnetAlert;
    private Alert hardwareAlert;
    private Alert bootDuringEnableAlert;
    private Alert undervoltageAlert;

    private CANcoderConfiguration config;

    public LoggedCANcoder(CANcoderIO io, String logPath, CANcoderConfiguration config) {
        this.io = io;
        this.logPath = logPath;
        this.config = config;

        disconnectedAlert = new Alert(logPath + " is disconnected", AlertType.kError);

        badMagnetAlert = new Alert(logPath + " has a bad magnet", AlertType.kWarning);
        hardwareAlert = new Alert(logPath + " encountered a hardware fault", AlertType.kWarning);
        bootDuringEnableAlert = new Alert(logPath + " booted during enable", AlertType.kWarning);
        undervoltageAlert = new Alert(logPath + " has insufficient voltage", AlertType.kWarning);

        updateConfig();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);

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
