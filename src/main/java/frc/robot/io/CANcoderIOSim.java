package frc.robot.io;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

import com.ctre.phoenix6.sim.CANcoderSimState;

// Interfaces with a simulated CANcoder attached to a physics simulator
public class CANcoderIOSim extends CANcoderIOBase {
    // The physics simulator
    private LinearSystemSim<N2, N1, N2> mechSim;

    // Allows us to interface with the simulated physical CANcoder
    private CANcoderSimState encoderSim;

    // Ratio of encoder radians to mech output. Generally for rotating systems radiansPerMechOutput should be the
    // encoder:mechanism ratio, and for linear systems
    // radiansPerMechOutput should be the encoderRatio/drumRadius.
    private double radiansPerMechOutput;

    // Offset of the encoder in mechanism radians
    private double offset;

    public CANcoderIOSim(
            int encoderId, LinearSystemSim<N2, N1, N2> mechSim, double radiansPerMechOutput, double offset) {
        super(encoderId, radiansPerMechOutput, offset);
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
        // Get position and velocity from the physics simulator
        double mechPosition = mechSim.getOutput(0);
        double mechVelocity = mechSim.getOutput(1);

        // Convert from mech output to encoder radians, then to encoder rotations
        encoderSim.setRawPosition(Units.radiansToRotations((mechPosition - offset) * radiansPerMechOutput));
        encoderSim.setVelocity(Units.radiansToRotations(mechVelocity * radiansPerMechOutput));

        // No need to update the physics simulation as that's done in TalonFXIOSim
    }
}
