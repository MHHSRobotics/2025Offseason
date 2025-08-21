package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class SwerveModule {
    private MotorIO driveMotor;
    private MotorIO angleMotor;
    private EncoderIO angleEncoder;

    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private Alert driveMotorDisconnect;
    private Alert driveMotorHardwareFault;
    private Alert driveMotorOverheat;

    private Alert angleMotorDisconnect;
    private Alert angleMotorHardwareFault;
    private Alert angleMotorOverheat;

    private Alert encoderDisconnect;
    private Alert encoderHardwareFault;
    private Alert encoderMagnetFault;

    private int index;

    public SwerveModule(
            MotorIO driveMotorIO,
            MotorIO angleMotorIO,
            EncoderIO angleEncoderIO,
            int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        this.index = index;

        driveMotor = driveMotorIO;
        angleMotor = angleMotorIO;
        angleEncoder = angleEncoderIO;

        driveMotor.setBraking(true);
        driveMotor.setGains(constants.DriveMotorGains);
        driveMotor.setGearRatio(constants.DriveMotorGearRatio);
        driveMotor.setStatorCurrentLimit(constants.SlipCurrent);
        driveMotor.setInverted(constants.DriveMotorInverted);

        angleMotor.setBraking(true);
        angleMotor.setGains(constants.SteerMotorGains);
        angleMotor.connectCANcoder(constants.EncoderId, constants.SteerMotorGearRatio, 1);
        angleMotor.setContinuousWrap(true);
        angleMotor.setInverted(constants.SteerMotorInverted);

        angleEncoder.setRatioAndOffset(1, constants.EncoderOffset);
        angleEncoder.setInverted(constants.EncoderInverted);

        String modulePos = "[UNKNOWN]";
        switch (index) {
            case 0:
                modulePos = "FL";
                break;
            case 1:
                modulePos = "FR";
                break;
            case 2:
                modulePos = "BL";
                break;
            case 3:
                modulePos = "BR";
                break;
        }

        driveMotorDisconnect =
                new Alert("The drive motor on module " + modulePos + " is disconnected", AlertType.kError);
        driveMotorHardwareFault = new Alert(
                "The drive motor on module " + modulePos + " has encountered a hardware error", AlertType.kError);
        driveMotorOverheat =
                new Alert("The drive motor on module " + modulePos + " is overheating", AlertType.kWarning);

        angleMotorDisconnect =
                new Alert("The angle motor on module " + modulePos + " is disconnected", AlertType.kError);
        angleMotorHardwareFault = new Alert(
                "The angle motor on module " + modulePos + " has encountered a hardware error", AlertType.kError);
        angleMotorOverheat =
                new Alert("The angle motor on module " + modulePos + " is overheating", AlertType.kWarning);

        encoderDisconnect = new Alert("The encoder on module " + modulePos + " is disconnect", AlertType.kError);
        encoderHardwareFault =
                new Alert("The encoder on module " + modulePos + " has encountered a hardware fault", AlertType.kError);
        encoderMagnetFault =
                new Alert("The encoder on module " + modulePos + " has a non-functioning magnet", AlertType.kError);
    }

    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    public void setAngleVoltage(double voltage) {
        angleMotor.setVoltage(voltage);
    }

    // Sets the target velocity of this module to the given radians per second value
    public void setDriveVelocity(double radPerSec) {
        driveMotor.setVelocityWithVoltage(radPerSec);
    }

    // Sets the target angle of this module (in radians)
    public void setAnglePosition(double position) {
        angleMotor.setGoalWithVoltage(position);
    }

    // Returns angle in radians
    public double getAngle() {
        return angleMotor.getInputs().position;
    }

    public double getPositionMeters() {
        return getWheelPosition() * constants.WheelRadius;
    }

    public double getVelocityMetersPerSec() {
        return driveMotor.getInputs().velocity * constants.WheelRadius;
    }

    public double getWheelPosition() {
        return driveMotor.getInputs().position;
    }

    // A SwerveModuleState represents a velocity and angle for the swerve module. This method sets the goal of the
    // swerve module to the given state.
    public void runSetpoint(SwerveModuleState state) {
        // Optimizes the state so that the wheel doesn't have to turn so much. I.e. instead of turning 180 degrees and
        // moving forward, the wheel can simply move backward
        state.optimize(Rotation2d.fromRadians(getAngle()));

        // Scales the drive velocity goal according to how far away the steer motor is from its goal. This makes driving
        // smoother.
        state.cosineScale(Rotation2d.fromRadians(getAngle()));
        // System.out.println(state.angle.getRadians());
        setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        setAnglePosition(state.angle.getRadians());
        // System.out.println(angleMotor.getInputs().setpoint);
    }

    // Runs a characterization for SysId
    public void runCharacterization(double volts) {
        setDriveVoltage(volts);
        setAnglePosition(0);
    }

    // Disables all motors in this module
    public void stop() {
        setDriveVoltage(0);
        setAngleVoltage(0);
    }

    // Returns the swerve module position, has drive and angle positions
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), Rotation2d.fromRadians(getAngle()));
    }

    // Gets the current modules state, includes drive velocity and angle
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), Rotation2d.fromRadians(getAngle()));
    }

    public void periodic() {
        driveMotor.updateInputs();
        angleMotor.updateInputs();
        angleEncoder.updateInputs();

        Logger.processInputs("Drive/Module" + index + "/DriveMotor", driveMotor.getInputs());
        Logger.processInputs("Drive/Module" + index + "/AngleMotor", angleMotor.getInputs());
        Logger.processInputs("Drive/Module" + index + "/AngleEncoder", angleEncoder.getInputs());

        driveMotorDisconnect.set(!driveMotor.getInputs().connected);
        driveMotorHardwareFault.set(driveMotor.getInputs().hardwareFault);
        driveMotorOverheat.set(driveMotor.getInputs().tempFault);

        angleMotorDisconnect.set(!angleMotor.getInputs().connected);
        angleMotorHardwareFault.set(angleMotor.getInputs().hardwareFault);
        angleMotorOverheat.set(angleMotor.getInputs().tempFault);

        encoderDisconnect.set(!angleEncoder.getInputs().connected);
        encoderHardwareFault.set(angleEncoder.getInputs().hardwareFault);
        encoderMagnetFault.set(angleEncoder.getInputs().badMagnetFault);
    }
}
