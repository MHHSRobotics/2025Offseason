package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedCANcoder;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;

public class SwerveModule {
    private LoggedTalonFX driveMotor;
    private LoggedTalonFX angleMotor;
    private LoggedCANcoder angleEncoder;

    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    public SwerveModule(
            TalonFXIO driveMotorIO,
            TalonFXIO angleMotorIO,
            CANcoderIO angleEncoderIO,
            int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;

        driveMotor = new LoggedTalonFX(driveMotorIO, "Swerve/Module" + index + "/DriveMotor");
        angleMotor = new LoggedTalonFX(angleMotorIO, "Swerve/Module" + index + "/AngleMotor");
        angleEncoder = new LoggedCANcoder(angleEncoderIO, "Swerve/Module" + index + "/AngleEncoder");

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
        return angleMotor.getPosition();
    }

    public double getPositionMeters() {
        return driveMotor.getPosition() * constants.WheelRadius;
    }

    public double getVelocityMetersPerSec() {
        return driveMotor.getVelocity() * constants.WheelRadius;
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

        setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        setAnglePosition(state.angle.getRadians());
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
}
