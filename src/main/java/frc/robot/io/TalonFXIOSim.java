package frc.robot.io;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class TalonFXIOSim extends TalonFXIOBase{
    private LinearSystemSim<N2,N1,N2> mechSim;
    private TalonFXSimState motorSim;
    private double radiansPerMechOutput;

    public TalonFXIOSim(int motorId,LinearSystemSim<N2,N1,N2> mechSim,double radiansPerMechOutput){
        super(motorId);
        this.mechSim=mechSim;
        // this should be gearRatio for arms and flywheels, gearRatio/drumRadius for elevators
        this.radiansPerMechOutput=radiansPerMechOutput;

        motorSim=getMotor().getSimState();
    }

    @Override
    public void updateSimulation(){
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        mechSim.setInput(motorSim.getMotorVoltage());

        mechSim.update(0.02); // 20 ms simulation step
       
        double mechPosition=mechSim.getOutput(0);
        double mechVelocity=mechSim.getOutput(1);
        motorSim.setRawRotorPosition(Units.radiansToRotations(mechPosition*radiansPerMechOutput));
        motorSim.setRotorVelocity(Units.radiansToRotations(mechVelocity*radiansPerMechOutput));
    }
}
