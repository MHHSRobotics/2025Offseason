package frc.robot.io;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.io.TalonFXIO.TalonFXIOInputs;

import static edu.wpi.first.units.Units.Radians;

// Manages alerts, logging, and tuning of a TalonFXIO
public class LoggedTalonFX {
    // NetworkTables path to log to
    private String logPath;

    // The TalonFX
    private final TalonFXIO io;

    // Current inputs from the TalonFXIO
    private final TalonFXIOInputsAutoLogged inputs = new TalonFXIOInputsAutoLogged();

    // Control objects
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private VoltageOut voltage = new VoltageOut(0);
    // private MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0);
    private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private Follower follower = new Follower(0, false);

    // Alerts for faults. These will appear on AdvantageScope/Elastic
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

    // Tunable numbers for PID, feedforward, and MotionMagic
    private LoggedNetworkNumber kP;
    private LoggedNetworkNumber kI;
    private LoggedNetworkNumber kD;
    private LoggedNetworkNumber kS;
    private LoggedNetworkNumber kG;
    private LoggedNetworkNumber kV;
    private LoggedNetworkNumber kA;
    private LoggedNetworkNumber cruiseVelocity;
    private LoggedNetworkNumber maxAccel;

    // Variables to check if tunable numbers have changed
    private double lastkP;
    private double lastkI;
    private double lastkD;
    private double lastkG;
    private double lastkS;
    private double lastkV;
    private double lastkA;
    private double lastCruiseVelocity;
    private double lastAccel;

    // Current TalonFX config
    private TalonFXConfiguration config;

    public LoggedTalonFX(TalonFXIO io, String logPath, TalonFXConfiguration config) {
        this.io = io;
        this.logPath = logPath;
        this.config = config;

        // Initialize alerts
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


        // Initialize tunable constants
        String pidPath = logPath.split("/")[0] + "Settings";
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

        // Apply the config to the TalonFX
        updateConfig();
    }

    public void follow(int motorId, boolean invert) {
        io.setControl(follower.withMasterID(motorId).withOpposeMasterDirection(invert));
    }

    public void periodic() {
        // Get the new inputs from TalonFXIO and log them
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);

        // Display any alerts that are currently active
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

        // Update the config if any tunable number has changed. If we update the config every frame then the bot will
        // lag
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

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public double getGoal() {
        return inputs.setpoint;
    }

    public TalonFXIOInputs getInputs() {
        return inputs;
    }

    public void setSpeed(double value) {
        io.setControl(dutyCycle.withOutput(value));
    }

    public void setVoltage(double volts) {
        io.setControl(voltage.withOutput(volts));
    }

    // public void setGoal(double position) {
    //     io.setControl(motionMagic.withPosition(Radians.of(position)));
    // }

    public void setGoal(double position) {
        io.setControl(motionMagic.withPosition(Radians.of(position)));
    }

    // Applies the current config
    private void updateConfig() {
        io.applyConfig(config);
    }
}
