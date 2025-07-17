package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.ArmCommands;
import frc.robot.io.CANcoderIO;
import frc.robot.io.CANcoderIOBase;
import frc.robot.io.TalonFXIO;
import frc.robot.io.TalonFXIOBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSim;

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
        // Create variables for all motors and encoders
        TalonFXIO armMotor;
        CANcoderIO armEncoder;
        switch (Constants.currentMode) {
            case REAL:
                // On a real bot, the arm should be using IO that interfaces with real motors (TalonFXIOBase,
                // CANcoderIObase)
                armMotor = new TalonFXIOBase(Arm.Constants.motorId, "rio");
                armEncoder = new CANcoderIOBase(Arm.Constants.encoderId, "rio");
                arm = new Arm(armMotor, armEncoder);
                break;

            case SIM:
                TalonFXIOBase _armMotor = new TalonFXIOBase(Arm.Constants.motorId);
                CANcoderIOBase _armEncoder = new CANcoderIOBase(Arm.Constants.encoderId);
                armMotor = _armMotor;
                armEncoder = _armEncoder;

                // Arm interfaces with simulated TalonFX and CANcoders connected to the physics simulator
                new ArmSim(_armMotor, _armEncoder);
                break;

            default:
                armMotor = new TalonFXIO();
                armEncoder = new CANcoderIO();
                break;
        }

        arm = new Arm(armMotor, armEncoder);
    }

    private void configureBindings() {
        // PID-based forward movement (CCW)
        controller.cross().and(() -> !Arm.Constants.manualArm.get()).onTrue(armCommands.setGoal(() -> 0));

        // Manual forward movement (CCW)
        controller
                .cross()
                .and(() -> Arm.Constants.manualArm.get())
                .onTrue(armCommands.setSpeed(() -> 0.2))
                .onFalse(armCommands.stop());

        // PID-based backward movement (CW)
        controller.circle().and(() -> !Arm.Constants.manualArm.get()).onTrue(armCommands.setGoal(() -> 1.5));

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
