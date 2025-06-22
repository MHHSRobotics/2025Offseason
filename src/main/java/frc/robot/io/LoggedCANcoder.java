package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.Logger;

public class LoggedCANcoder {
    private String logPath;

    private final CANcoderIO io;
    private final CANcoderIOInputsAutoLogged inputs = new CANcoderIOInputsAutoLogged();

    private Alert disconnectedAlert;

    private Alert badMagnetAlert;
    private Alert hardwareAlert;
    private Alert bootDuringEnableAlert;
    private Alert undervoltageAlert;

    public LoggedCANcoder(CANcoderIO io, String logPath) {
        this.io = io;
        this.logPath = logPath;

        disconnectedAlert = new Alert(logPath + " is disconnected", AlertType.kError);

        badMagnetAlert = new Alert(logPath + " has a bad magnet", AlertType.kWarning);
        hardwareAlert = new Alert(logPath + " encountered a hardware fault", AlertType.kWarning);
        bootDuringEnableAlert = new Alert(logPath + " booted during enable", AlertType.kWarning);
        undervoltageAlert = new Alert(logPath + " has insufficient voltage", AlertType.kWarning);
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
}
