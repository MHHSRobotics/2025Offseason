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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

    // Controller for SysId commands
    private final CommandPS5Controller sysIdController = new CommandPS5Controller(2);

    public RobotContainer() {
        // Initialize all the IO objects, subsystems, and mechanism simulators
        initSubsystems();
        armCommands = new ArmCommands(arm);

        // Add controller bindings
        configureBindings();

        // Add SysId bindings
        configureSysId();
    }

    private void initSubsystems() {
        switch (Constants.currentMode) {
            case REAL:
                // On a real bot, the arm should be using IO that interfaces with real motors (TalonFXIOBase,
                // CANcoderIObase)
                arm = new Arm(
                        new TalonFXIOBase(Arm.Constants.motorId, "rio"),
                        new CANcoderIOBase(
                                Arm.Constants.encoderId,
                                "rio",
                                Arm.Constants.encoderRatio,
                                Arm.Constants.encoderOffset));
                break;

            case SIM:
                // In simulation, create the physics simulator for the arm
                LinearSystemSim<N2, N1, N2> armMech = new SingleJointedArmSim(
                        DCMotor.getKrakenX60(1),
                        Arm.Constants.gearRatio,
                        Arm.Constants.moi,
                        Arm.Constants.armLength,
                        Arm.Constants.minAngle,
                        Arm.Constants.maxAngle,
                        true,
                        Arm.Constants.startAngle);

                // Arm interfaces with simulated TalonFX and CANcoders connected to the physics simulator
                arm = new Arm(
                        new TalonFXIOSim(Arm.Constants.motorId, armMech, Arm.Constants.gearRatio),
                        new CANcoderIOSim(
                                Arm.Constants.encoderId,
                                armMech,
                                Arm.Constants.encoderRatio,
                                Arm.Constants.encoderOffset));
                break;

            default:
                // In replay, arm doesn't interface with anything
                arm = new Arm(new TalonFXIO(), new CANcoderIO());
                break;
        }
    }

    private void configureBindings() {
        // PID-based forward movement (CCW)
        controller
                .cross()
                .and(() -> !Arm.Constants.manualArm.get())
                .whileTrue(new RepeatCommand(armCommands.changeGoal(() -> 0.02)));

        // Manual forward movement (CCW)
        controller
                .cross()
                .and(() -> Arm.Constants.manualArm.get())
                .onTrue(armCommands.setSpeed(() -> 0.2))
                .onFalse(armCommands.stop());

        // PID-based backward movement (CW)
        controller
                .circle()
                .and(() -> !Arm.Constants.manualArm.get())
                .whileTrue(new RepeatCommand(armCommands.changeGoal(() -> -0.02)));

        // Manual backward movement (CW)
        controller
                .circle()
                .and(() -> Arm.Constants.manualArm.get())
                .onTrue(armCommands.setSpeed(() -> -0.2))
                .onFalse(armCommands.stop());
    }

    private void configureSysId() {
        sysIdController.cross().whileTrue(armCommands.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysIdController.circle().whileTrue(armCommands.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysIdController.square().whileTrue(armCommands.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysIdController.triangle().whileTrue(armCommands.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
