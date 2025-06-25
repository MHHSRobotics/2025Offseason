package frc.robot;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
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
                arm = new Arm(
                        new TalonFXIOBase(Arm.Constants.motorId, "rio"),
                        new CANcoderIOBase(
                                Arm.Constants.encoderId,
                                "rio",
                                Arm.Constants.encoderRatio,
                                Arm.Constants.encoderOffset));
                break;
            case SIM:
                LinearSystemSim<N2, N1, N2> armMech = new SingleJointedArmSim(
                        DCMotor.getKrakenX60(1),
                        Arm.Constants.gearRatio,
                        Arm.Constants.moi,
                        Arm.Constants.armLength,
                        Arm.Constants.minAngle,
                        Arm.Constants.maxAngle,
                        true,
                        Arm.Constants.startAngle);
                arm = new Arm(
                        new TalonFXIOSim(Arm.Constants.motorId, armMech, Arm.Constants.gearRatio),
                        new CANcoderIOSim(
                                Arm.Constants.encoderId,
                                armMech,
                                Arm.Constants.encoderRatio,
                                Arm.Constants.encoderOffset));
                break;
            default:
                arm = new Arm(new TalonFXIO(), new CANcoderIO());
                break;
        }
    }

    private void configureBindings() {
        controller
                .cross()
                .and(() -> !Arm.Constants.manualArm.get())
                .whileTrue(new RepeatCommand(armCommands.changeGoal(() -> 0.02)));
        controller
                .cross()
                .and(() -> Arm.Constants.manualArm.get())
                .onTrue(armCommands.setSpeed(() -> 0.2))
                .onFalse(armCommands.stop());
        controller
                .circle()
                .and(() -> !Arm.Constants.manualArm.get())
                .whileTrue(new RepeatCommand(armCommands.changeGoal(() -> -0.02)));
        controller
                .circle()
                .and(() -> Arm.Constants.manualArm.get())
                .onTrue(armCommands.setSpeed(() -> -0.2))
                .onFalse(armCommands.stop());
        // controller.cross().onTrue(armCommands.setSpeed(() -> 1)).onFalse(armCommands.setSpeed(() -> -1));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
