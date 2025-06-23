package frc.robot.io;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

    private LoggedNetworkNumber kP;
    private LoggedNetworkNumber kI;
    private LoggedNetworkNumber kD;
    private LoggedNetworkNumber kS;
    private LoggedNetworkNumber kG;
    private LoggedNetworkNumber kV;
    private LoggedNetworkNumber kA;
    private LoggedNetworkNumber cruiseVelocity;
    private LoggedNetworkNumber maxAccel;

    private double lastkP;
    private double lastkI;
    private double lastkD;
    private double lastkG;
    private double lastkS;
    private double lastkV;
    private double lastkA;
    private double lastCruiseVelocity;
    private double lastAccel;

    private TalonFXConfiguration config;

    public LoggedTalonFX(TalonFXIO io, String logPath, TalonFXConfiguration config) {
        this.io = io;
        this.logPath = logPath;
        this.config = config;

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

        String pidPath = "AdvantageKit/" + logPath.split("/")[0] + "/" + logPath.split("/")[0] + "PID";
        kP = new LoggedNetworkNumber(pidPath + "/kP", config.Slot0.kP);
        kI = new LoggedNetworkNumber(pidPath + "/kI", config.Slot0.kI);
        kD = new LoggedNetworkNumber(pidPath + "/kD", config.Slot0.kD);
        kG = new LoggedNetworkNumber(pidPath + "/kG", config.Slot0.kG);
        kS = new LoggedNetworkNumber(pidPath + "/kS", config.Slot0.kS);
        kV = new LoggedNetworkNumber(pidPath + "/kV", config.Slot0.kV);
        kA = new LoggedNetworkNumber(pidPath + "/kA", config.Slot0.kA);
        cruiseVelocity =
                new LoggedNetworkNumber(pidPath + "/cruiseVelocity", config.MotionMagic.MotionMagicCruiseVelocity);
        maxAccel = new LoggedNetworkNumber(pidPath + "/maxAccel", config.MotionMagic.MotionMagicAcceleration);

        updateConfig();
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

        if (kP.get() != lastkP
                || kI.get() != lastkI
                || kD.get() != lastkD
                || kG.get() != lastkG
                || kS.get() != lastkS
                || kV.get() != lastkV
                || kA.get() != lastkA
                || cruiseVelocity.get() != lastCruiseVelocity
                || maxAccel.get() != lastAccel) {
            updateConfig();
        }
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

    // Loads the data from the tunable values into config and applies it, also updates the last tunable values
    private void updateConfig() {
        lastkP = kP.get();
        lastkI = kI.get();
        lastkD = kD.get();
        lastkG = kG.get();
        lastkS = kS.get();
        lastkV = kV.get();
        lastkA = kA.get();
        lastCruiseVelocity = cruiseVelocity.get();
        lastAccel = maxAccel.get();

        config.Slot0.kP = Units.rotationsToRadians(kP.get());
        config.Slot0.kI = Units.rotationsToRadians(kI.get());
        config.Slot0.kD = Units.rotationsToRadians(kD.get());
        config.Slot0.kG = kG.get();
        config.Slot0.kS = kS.get();
        config.Slot0.kV = Units.rotationsToRadians(kV.get());
        config.Slot0.kA = Units.rotationsToRadians(kA.get());
        config.MotionMagic.MotionMagicCruiseVelocity = Units.rotationsToRadians(cruiseVelocity.get());
        config.MotionMagic.MotionMagicAcceleration = Units.rotationsToRadians(maxAccel.get());
        io.applyConfig(config);
    }
}
