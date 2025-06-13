package frc.robot.subsystems;

import frc.robot.io.TalonFXIO;

public class Arm {
    private TalonFXIO motor;
    public Arm(TalonFXIO motor){
        this.motor=motor;
    }

    public void set(double value){
        motor.set(value);
    }

    public void setPosition(double pos){
        motor.setPosition(pos);
    }

}
