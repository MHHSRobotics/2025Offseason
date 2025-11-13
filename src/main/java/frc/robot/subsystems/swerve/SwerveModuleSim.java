package frc.robot.subsystems.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class SwerveModuleSim extends SubsystemBase {
    private static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor steerGearbox = DCMotor.getKrakenX60Foc(1);

    private MotorIO driveMotor;
    private MotorIO steerMotor;
    private EncoderIO steerEncoder;

    private DCMotorSim driveMech;
    private DCMotorSim steerMech;

    public SwerveModuleSim(
            MotorIO driveMotorIO,
            MotorIO steerMotorIO,
            EncoderIO steerEncoderIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        driveMotor = driveMotorIO;
        steerMotor = steerMotorIO;
        steerEncoder = steerEncoderIO;
        driveMech = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(driveGearbox, constants.DriveInertia, constants.DriveMotorGearRatio),
                driveGearbox);
        steerMech = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(steerGearbox, constants.SteerInertia, constants.SteerMotorGearRatio),
                steerGearbox);
    }

    @Override
    public void periodic() {
        driveMech.setInputVoltage(driveMotor.getInputs().appliedVoltageVolts);
        steerMech.setInputVoltage(steerMotor.getInputs().appliedVoltageVolts);

        driveMech.update(0.02);
        steerMech.update(0.02);

        driveMotor.setMechPosition(driveMech.getAngularPositionRad());
        driveMotor.setMechVelocity(driveMech.getAngularVelocityRadPerSec());

        double steerAngle = steerMech.getAngularPositionRad();
        double steerVelocity = steerMech.getAngularVelocityRadPerSec();

        steerMotor.setMechPosition(steerAngle);
        steerMotor.setMechVelocity(steerVelocity);

        steerEncoder.setMechPosition(steerAngle);
        steerEncoder.setMechVelocity(steerVelocity);
    }
}
