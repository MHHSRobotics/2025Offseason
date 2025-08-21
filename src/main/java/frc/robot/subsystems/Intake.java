package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.MotorIO;

public class Intake extends SubsystemBase {
    private MotorIO motor;

    public static class Constants {
        public static final int motorId = 24;
        public static final double intakeSpeed = 0.9;
        public static final double outtakeTime = 0.5;
        public static final double intakeTime = 0.5;
        public static boolean motorInverted = false;
    }

    public Intake(MotorIO motorIO) {
        motor = motorIO;
        motor.setInverted(Constants.motorInverted);
    }

    public void setSpeed(double value) {
        motor.setSpeed(value);
    }

    public void stopMotor() {
        motor.setSpeed(0);
    }

    @Override
    public void periodic() {
        motor.updateInputs();
    }
}
