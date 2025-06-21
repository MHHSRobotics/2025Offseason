package frc.robot.io;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

import com.ctre.phoenix6.sim.CANcoderSimState;

public class CANcoderIOSim extends CANcoderIOBase {
    private LinearSystemSim<N2, N1, N2> mechSim;

    private CANcoderSimState encoderSim;

    private double radiansPerMechOutput;

    private double offset;

    public CANcoderIOSim(
            int encoderId, LinearSystemSim<N2, N1, N2> mechSim, double radiansPerMechOutput, double offset) {
        super(encoderId);
        this.mechSim = mechSim;
        this.radiansPerMechOutput = radiansPerMechOutput;
        this.offset = offset;
        encoderSim = getCANcoder().getSimState();
    }

    public CANcoderIOSim(int encoderId, LinearSystemSim<N2, N1, N2> mechSim, double radiansPerMechOutput) {
        this(encoderId, mechSim, radiansPerMechOutput, 0);
    }

    @Override
    public void updateSimulation() {
        double mechPosition = mechSim.getOutput(0);
        double mechVelocity = mechSim.getOutput(1);

        encoderSim.setRawPosition(Units.radiansToRotations((mechPosition - offset) * radiansPerMechOutput));
        encoderSim.setVelocity(Units.radiansToRotations(mechVelocity * radiansPerMechOutput));
    }
}
