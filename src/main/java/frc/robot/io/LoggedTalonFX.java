package frc.robot.io;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;

public class LoggedTalonFX {
    private String logPath;
    private final TalonFXIO io;
    private final TalonFXIOInputsAutoLogged inputs = new TalonFXIOInputsAutoLogged();

    private DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0);

    private Alert disconnectedAlert;

    private Alert hardwareAlert;
    private Alert procTempAlert;
    private Alert deviceTempAlert;
    private Alert undervoltageAlert;
    private Alert bootDuringEnableAlert;
    private Alert forwardHardLimitAlert;
    private Alert forwardSoftLimitAlert;
    private Alert reverseHardLimitAlert;
    private Alert reverseSoftLimitAlert;

    public LoggedTalonFX(TalonFXIO io, String logPath) {
        this.io = io;
        this.logPath = logPath;

        disconnectedAlert = new Alert(logPath + " is disconnected", AlertType.kError);

        hardwareAlert = new Alert(logPath + " encountered a hardware fault", AlertType.kWarning);
        procTempAlert = new Alert(logPath + " processor is overheating", AlertType.kWarning);
        deviceTempAlert = new Alert(logPath + " is overheating", AlertType.kWarning);
        undervoltageAlert = new Alert(logPath + " has insufficient voltage", AlertType.kWarning);
        bootDuringEnableAlert = new Alert(logPath + " booted during enable", AlertType.kWarning);
        forwardHardLimitAlert = new Alert(logPath + " reached its forward hard limit", AlertType.kWarning);
        forwardSoftLimitAlert = new Alert(logPath + " reached its forward soft limit", AlertType.kWarning);
        reverseHardLimitAlert = new Alert(logPath + " reached its reverse hard limit", AlertType.kWarning);
        reverseSoftLimitAlert = new Alert(logPath + " reached its reverse soft limit", AlertType.kWarning);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);
        disconnectedAlert.set(!inputs.connected);

        hardwareAlert.set(inputs.hardwareFault);
        procTempAlert.set(inputs.procTempFault);
        deviceTempAlert.set(inputs.deviceTempFault);
        undervoltageAlert.set(inputs.undervoltageFault);
        bootDuringEnableAlert.set(inputs.bootDuringEnable);
        forwardHardLimitAlert.set(inputs.forwardHardLimit);
        forwardSoftLimitAlert.set(inputs.forwardSoftLimit);
        reverseHardLimitAlert.set(inputs.reverseHardLimit);
        reverseSoftLimitAlert.set(inputs.reverseSoftLimit);
    }

    public double getPositionRad() {
        return inputs.positionRad;
    }

    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }

    public double getGoal() {
        return inputs.setpointRad;
    }

    public void setSpeed(double value) {
        io.setControl(dutyCycle.withOutput(value));
    }

    public void setGoal(double position) {
        io.setControl(motionMagic.withPosition(Radians.of(position)));
    }
}
