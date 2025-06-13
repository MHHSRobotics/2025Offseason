package frc.robot.io;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXIOReal implements TalonFXIO{
    private TalonFX motor;
    private TalonFXIOInputsAutoLogged motorInputs;
    public TalonFXIOReal(int motorId,String canBus){
        motor=new TalonFX(motorId, canBus);
    }

}
