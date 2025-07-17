package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.CANcoderIO.CANcoderIOInputs;

import static edu.wpi.first.units.Units.Radians;

// Manages alerts, logging, and tuning of a CANcoderIO
public class LoggedCANcoder {
    // NetworkTables path to log to
    private String logPath;

    // The CANcoder
    private final CANcoderIO io;

    // Current inputs from the CANcoderIO
    private final CANcoderIOInputsAutoLogged inputs = new CANcoderIOInputsAutoLogged();

    // Alerts for faults. These will appear on AdvantageScope/Elastic
    private Alert disconnectedAlert;

    private Alert badMagnetAlert;
    private Alert hardwareAlert;
    private Alert bootDuringEnableAlert;
    private Alert undervoltageAlert;

    // Current CANcoder config
    private CANcoderConfiguration config;

    // Whether the config has changed since the last update
    private boolean configChanged = true;

    public LoggedCANcoder(CANcoderIO io, String logPath) {
        this.io = io;
        this.logPath = logPath;

        config = new CANcoderConfiguration();

        // Initialize alerts
        disconnectedAlert = new Alert(logPath + " is disconnected", AlertType.kError);

        badMagnetAlert = new Alert(logPath + " has a bad magnet", AlertType.kWarning);
        hardwareAlert = new Alert(logPath + " encountered a hardware fault", AlertType.kWarning);
        bootDuringEnableAlert = new Alert(logPath + " booted during enable", AlertType.kWarning);
        undervoltageAlert = new Alert(logPath + " has insufficient voltage", AlertType.kWarning);
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

        // If the config was changed, apply it
        if (configChanged) {
            io.applyConfig(config);
            configChanged = false;
        }
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

    public void setRatioAndOffset(double ratio, double offset) {
        io.setEncoderRatio(ratio);
        config.MagnetSensor.withMagnetOffset(Radians.of(offset * ratio));
        configChanged = true;
    }

    public void setInverted(boolean inverted) {
        config.MagnetSensor.SensorDirection =
                inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        configChanged = true;
    }
}
