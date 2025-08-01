package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

// This class simulates the physical arm on the robot attached to a motor and an encoder.
public class ArmSim extends SubsystemBase {
    // Sim states for the motor and encoder
    private MotorIO motor;
    private EncoderIO encoder;

    private static final DCMotor armGearbox = DCMotor.getKrakenX60Foc(1);
    // The simulation model for the arm. This has one input, the voltage applied by the motor, and two outputs, the
    // position and velocity of the arm.
    private SingleJointedArmSim armMech;

    public ArmSim(MotorIO motor, EncoderIO encoder) {
        this.motor = motor;
        this.encoder = encoder;
        armMech = new SingleJointedArmSim(
                armGearbox,
                Arm.Constants.gearRatio,
                Arm.Constants.moi,
                Arm.Constants.armLength,
                Arm.Constants.minAngle,
                Arm.Constants.maxAngle,
                true,
                Arm.Constants.startAngle);
    }

    @Override
    public void periodic() {
        // Update the voltage input to the arm
        armMech.setInputVoltage(motor.getInputs().appliedVoltage);

        // Step forward 20ms (default robot loop duration)
        armMech.update(0.02);

        // Set position and velocity of the motor and encoder. These have to be converted from radians to rotations
        // because the TalonFX doesn't like radians
        motor.setMechPosition(armMech.getAngleRads());
        motor.setMechVelocity(armMech.getVelocityRadPerSec());
        encoder.setMechPosition(armMech.getAngleRads() - Arm.Constants.encoderOffset);
        encoder.setMechVelocity(armMech.getVelocityRadPerSec());
    }
}
