package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;

public class Intake extends SubsystemBase{
    private LoggedTalonFX motor;
    
    public static class Constants {
        public static final int motorId = 24;
        public static final double intakeSpeed = 0.9;
        public static final double outtakeTime = 0.5;
        public static final double intakeTime = 0.5;
        public static boolean motorInverted = false;
    }

    public Intake(TalonFXIO motorIO){
        motor = new LoggedTalonFX(motorIO, "Intake/Motor");  
    }

    public void setSpeed(double value) {
        motor.setSpeed(value);
    }

    public void stopMotor(){
        motor.setSpeed(0);
    }

    @Override
    public void periodic(){
        motor.periodic();
    }
}