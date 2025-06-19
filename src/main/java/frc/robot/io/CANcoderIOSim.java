package frc.robot.io;

import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class CANcoderIOSim extends CANcoderIOBase{
    private LinearSystemSim<N2,N1,N2> mechSim;

    private CANcoderSimState encoderSim;

    private double radiansPerMechOutput;

    public CANcoderIOSim(int encoderId,LinearSystemSim<N2,N1,N2> mechSim,double radiansPerMechOutput){
        super(encoderId);
        this.mechSim=mechSim;
        this.radiansPerMechOutput=radiansPerMechOutput;
        encoderSim=getCANcoder().getSimState();
    }

    @Override
    public void updateSimulation(){
        double mechPosition=mechSim.getOutput(0);
        double mechVelocity=mechSim.getOutput(1);

        encoderSim.setRawPosition(Units.radiansToRotations(mechPosition*radiansPerMechOutput));
        encoderSim.setVelocity(Units.radiansToRotations(mechVelocity*radiansPerMechOutput));
    }
}
