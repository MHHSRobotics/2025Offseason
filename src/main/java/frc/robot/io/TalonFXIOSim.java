package frc.robot.io;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

import com.ctre.phoenix6.sim.TalonFXSimState;

// Interfaces with a simulated TalonFX attached to a physics simulator
public class TalonFXIOSim extends TalonFXIOBase {
    // Physics simulator of whatever the motor is attached to
    private LinearSystemSim<N2, N1, N2> mechSim;

    // Allows us to give inputs to the TalonFX from the simulated mechanism
    private TalonFXSimState motorSim;

    // Ratio of motor radians to mechanism output. Mechanism output can be in radians (for arms) or meters (for
    // elevators). Generally for rotating systems tradiansPerMechOutput should be the gear ratio, and for linear systems
    // radiansPerMechOutput should be gearRatio/drumRadius.
    private double radiansPerMechOutput;

    // Offset in mech output units
    private double offset;

    public TalonFXIOSim(int motorId, LinearSystemSim<N2, N1, N2> mechSim, double radiansPerMechOutput, double offset) {
        super(motorId);
        this.mechSim = mechSim;

        this.radiansPerMechOutput = radiansPerMechOutput;

        this.offset = offset;

        motorSim = getMotor().getSimState();
    }

    public TalonFXIOSim(int motorId, LinearSystemSim<N2, N1, N2> mechSim, double radiansPerMechOutput) {
        this(motorId, mechSim, radiansPerMechOutput, 0);
    }

    @Override
    public void updateSimulation() {
        // Supply motor with battery voltage
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Set the input to the physics simulator to the current voltage output of the TalonFX
        mechSim.setInput(motorSim.getMotorVoltage());

        mechSim.update(0.02); // 20 ms simulation step

        // Get position, velocity of the physics simulation
        double mechPosition = mechSim.getOutput(0);
        double mechVelocity = mechSim.getOutput(1);

        // Set the position and velocity read by the TalonFX to the mechanism, with an offset and scale
        motorSim.setRawRotorPosition(Units.radiansToRotations((mechPosition - offset) * radiansPerMechOutput));
        motorSim.setRotorVelocity(Units.radiansToRotations(mechVelocity * radiansPerMechOutput));
    }
}
