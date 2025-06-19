package frc.robot;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import frc.robot.commands.ArmCommands;
import frc.robot.io.CANcoderIO;
import frc.robot.io.CANcoderIOBase;
import frc.robot.io.CANcoderIOSim;
import frc.robot.io.TalonFXIO;
import frc.robot.io.TalonFXIOBase;
import frc.robot.io.TalonFXIOSim;
import frc.robot.subsystems.Arm;

public class RobotContainer {
    private Arm arm;
    private final ArmCommands armCommands;
    private final CommandPS5Controller controller = new CommandPS5Controller(0);

    public RobotContainer() {
        initSubsystems();
        armCommands = new ArmCommands(arm);
        configureBindings();
    }

    private void initSubsystems() {
        switch (Constants.currentMode) {
            case REAL:
                arm = new Arm(new TalonFXIOBase(Arm.motorId, "rio"), new CANcoderIOBase(Arm.encoderId, "rio"));
                break;
            case SIM:
                double gearRatio = Arm.rotorToSensorRatio * Arm.sensorToMechanismRatio;
                LinearSystemSim<N2, N1, N2> mechSim = new SingleJointedArmSim(
                        DCMotor.getKrakenX60(1),
                        gearRatio,
                        Arm.moi,
                        0,
                        Arm.minAngle,
                        Arm.maxAngle,
                        true,
                        Units.degreesToRadians(90));
                arm = new Arm(
                        new TalonFXIOSim(Arm.motorId, mechSim, gearRatio),
                        new CANcoderIOSim(Arm.encoderId, mechSim, gearRatio));
                break;
            default:
                arm = new Arm(new TalonFXIO(), new CANcoderIO());
                break;
        }
    }

    private void configureBindings() {
        controller.L1().whileTrue(new RepeatCommand(armCommands.changeGoal(() -> 0.02)));
        controller.R1().whileTrue(new RepeatCommand(armCommands.changeGoal(() -> -0.02)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
