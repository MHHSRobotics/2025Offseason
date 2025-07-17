package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.io.CANcoderIOBase;
import frc.robot.io.TalonFXIOBase;

// This class simulates the physical arm on the robot attached to a motor and an encoder.
public class ArmSim extends SubsystemBase {
    // Sim states for the motor and encoder
    private TalonFXSimState motor;
    private CANcoderSimState encoder;

    // The simulation model for the arm. This has one input, the voltage applied by the motor, and two outputs, the position and velocity of the arm.
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
        // Update the voltage input to the arm
        armMech.setInputVoltage(motor.getMotorVoltage());

        // Step forward 20ms (default robot loop duration)
        armMech.update(0.02);

        // Set position and velocity of the motor and encoder. These have to be converted from radians to rotations because the TalonFX doesn't like radians
        motor.setRawRotorPosition(Units.radiansToRotations(armMech.getAngleRads() * Arm.Constants.gearRatio));
        motor.setRotorVelocity(Units.radiansToRotations(armMech.getVelocityRadPerSec() * Arm.Constants.gearRatio));
        encoder.setRawPosition(Units.radiansToRotations(
                (armMech.getAngleRads() - Arm.Constants.encoderOffset) * Arm.Constants.encoderRatio));
        encoder.setVelocity(Units.radiansToRotations(armMech.getVelocityRadPerSec() * Arm.Constants.encoderRatio));
    }
}
