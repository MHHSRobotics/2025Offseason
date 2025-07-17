package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.io.CANcoderIOBase;
import frc.robot.io.TalonFXIOBase;

public class ArmSim extends SubsystemBase {
    private TalonFXSimState motor;
    private CANcoderSimState encoder;

    private SingleJointedArmSim armMech = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Arm.Constants.gearRatio,
            Arm.Constants.moi,
            Arm.Constants.armLength,
            Arm.Constants.minAngle,
            Arm.Constants.maxAngle,
            true,
            Arm.Constants.startAngle);

    public ArmSim(TalonFXIOBase motor, CANcoderIOBase encoder) {
        this.motor = motor.getMotor().getSimState();
        this.encoder = encoder.getCANcoder().getSimState();
    }

    @Override
    public void periodic() {
        armMech.setInputVoltage(motor.getMotorVoltage());
        armMech.update(0.02);
        motor.setRawRotorPosition(Units.radiansToRotations(armMech.getAngleRads() * Arm.Constants.gearRatio));
        motor.setRotorVelocity(Units.radiansToRotations(armMech.getVelocityRadPerSec() * Arm.Constants.gearRatio));
        encoder.setRawPosition(Units.radiansToRotations(
                (armMech.getAngleRads() - Arm.Constants.encoderOffset) * Arm.Constants.encoderRatio));
        encoder.setVelocity(Units.radiansToRotations(armMech.getVelocityRadPerSec() * Arm.Constants.encoderRatio));
    }
}
