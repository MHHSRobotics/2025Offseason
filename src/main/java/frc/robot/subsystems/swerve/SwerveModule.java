package frc.robot.subsystems.swerve;

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

    private int index;

    public SwerveModule(
            TalonFXIO driveMotorIO,
            TalonFXIO angleMotorIO,
            CANcoderIO angleEncoderIO,
            int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.index = index;
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
}
