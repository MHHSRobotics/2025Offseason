package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Constants.Mode;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.HangCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.WristCommands;
import frc.robot.io.EncoderIO;
import frc.robot.io.EncoderIOCANcoder;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIOTalonFX;
import frc.robot.network.RobotPublisher;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSubsystemSim;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.GyroSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleSim;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristSim;

public class RobotContainer {
    private Arm arm;
    private Elevator elevator;
    private Wrist wrist;
    private Hang hang;
    private Intake intake;
    private Swerve swerve;

    private ArmCommands armCommands;
    private ElevatorCommands elevatorCommands;
    private WristCommands wristCommands;
    private HangCommands hangCommands;
    private IntakeCommands intakeCommands;
    private SwerveCommands swerveCommands;

    // Main drive controller
    private final CommandPS5Controller controller = new CommandPS5Controller(0);

    // Controller for SysId commands (not working right now)
    // private final CommandPS5Controller sysIdController = new CommandPS5Controller(2);

    // Virtual controller for sim
    private final CommandPS5Controller testController = new CommandPS5Controller(3);
    private LoggedDashboardChooser<String> testControllerChooser;

    // Publishes all robot data to AdvantageScope
    private RobotPublisher publisher;

    public RobotContainer() {
        // Initialize all the IO objects, subsystems, and mechanism simulators
        initSubsystems();

        // Initialize command classes
        initCommands();

        // Add controller bindings
        configureBindings();

        // If in sim configure bindings for test controller
        if (Constants.currentMode == Mode.SIM) {
            configureTestBindings();
        }

        // Add SysId bindings
        configureSysId();

        // Initialize the publisher
        publisher = new RobotPublisher(arm, wrist, intake, elevator, hang, swerve);
    }

    private void initSubsystems() {
        // Initialize subsystems in order: arm, elevator, wrist, intake, hang, swerve
        // Each subsystem is created immediately after its motor/encoder initialization

        // Initialize arm motor and encoder
        MotorIO armMotor;
        EncoderIO armEncoder;
        switch (Constants.currentMode) {
            case REAL:
            case SIM:
                armMotor = new MotorIOTalonFX(Arm.Constants.motorId, Constants.defaultBus);
                armEncoder = new EncoderIOCANcoder(Arm.Constants.encoderId, Constants.defaultBus);
                break;
            default:
                armMotor = new MotorIO();
                armEncoder = new EncoderIO();
                break;
        }
        // Create arm subsystem
        arm = new Arm(armMotor, armEncoder);

        // Initialize elevator motors and encoder
        MotorIO elevatorLeftMotor;
        MotorIO elevatorRightMotor;
        EncoderIO elevatorEncoder;
        switch (Constants.currentMode) {
            case REAL:
            case SIM:
                elevatorLeftMotor = new MotorIOTalonFX(Elevator.Constants.leftMotorId, Constants.defaultBus);
                elevatorRightMotor = new MotorIOTalonFX(Elevator.Constants.rightMotorId, Constants.defaultBus);
                elevatorEncoder = new EncoderIOCANcoder(Elevator.Constants.encoderId, Constants.defaultBus);
                break;
            default:
                elevatorLeftMotor = new MotorIO();
                elevatorRightMotor = new MotorIO();
                elevatorEncoder = new EncoderIO();
                break;
        }
        // Create elevator subsystem
        elevator = new Elevator(elevatorLeftMotor, elevatorRightMotor, elevatorEncoder);

        // Initialize wrist motor and encoder
        MotorIO wristMotor;
        EncoderIO wristEncoder;
        switch (Constants.currentMode) {
            case REAL:
            case SIM:
                wristMotor = new MotorIOTalonFX(Wrist.Constants.motorId, Constants.defaultBus);
                wristEncoder = new EncoderIOCANcoder(Wrist.Constants.encoderId, Constants.defaultBus);
                break;
            default:
                wristMotor = new MotorIO();
                wristEncoder = new EncoderIO();
                break;
        }
        // Create wrist subsystem
        wrist = new Wrist(wristMotor, wristEncoder);

        // Initialize intake motor
        MotorIO intakeMotor;
        switch (Constants.currentMode) {
            case REAL:
            case SIM:
                intakeMotor = new MotorIOTalonFX(Intake.Constants.motorId, Constants.defaultBus);
                break;
            default:
                intakeMotor = new MotorIO();
                break;
        }
        // Create intake subsystem
        intake = new Intake(intakeMotor);

        // Initialize hang motor
        MotorIO hangMotor;
        switch (Constants.currentMode) {
            case REAL:
            case SIM:
                hangMotor = new MotorIOTalonFX(Hang.Constants.motorId, Constants.defaultBus);
                break;
            default:
                hangMotor = new MotorIO();
                break;
        }
        // Create hang subsystem
        hang = new Hang(hangMotor);

        // Initialize swerve motors, encoders, and gyro
        MotorIO flDriveMotor, flAngleMotor, frDriveMotor, frAngleMotor;
        MotorIO blDriveMotor, blAngleMotor, brDriveMotor, brAngleMotor;
        EncoderIO flEncoder, frEncoder, blEncoder, brEncoder;
        GyroIO gyro;
        switch (Constants.currentMode) {
            case REAL:
            case SIM:
                flDriveMotor = new MotorIOTalonFX(TunerConstants.FrontLeft.DriveMotorId, Constants.swerveBus);
                flAngleMotor = new MotorIOTalonFX(TunerConstants.FrontLeft.SteerMotorId, Constants.swerveBus);
                flEncoder = new EncoderIOCANcoder(TunerConstants.FrontLeft.EncoderId, Constants.swerveBus);

                frDriveMotor = new MotorIOTalonFX(TunerConstants.FrontRight.DriveMotorId, Constants.swerveBus);
                frAngleMotor = new MotorIOTalonFX(TunerConstants.FrontRight.SteerMotorId, Constants.swerveBus);
                frEncoder = new EncoderIOCANcoder(TunerConstants.FrontRight.EncoderId, Constants.swerveBus);

                blDriveMotor = new MotorIOTalonFX(TunerConstants.BackLeft.DriveMotorId, Constants.swerveBus);
                blAngleMotor = new MotorIOTalonFX(TunerConstants.BackLeft.SteerMotorId, Constants.swerveBus);
                blEncoder = new EncoderIOCANcoder(TunerConstants.BackLeft.EncoderId, Constants.swerveBus);

                brDriveMotor = new MotorIOTalonFX(TunerConstants.BackRight.DriveMotorId, Constants.swerveBus);
                brAngleMotor = new MotorIOTalonFX(TunerConstants.BackRight.SteerMotorId, Constants.swerveBus);
                brEncoder = new EncoderIOCANcoder(TunerConstants.BackRight.EncoderId, Constants.swerveBus);

                gyro = new GyroIOPigeon(TunerConstants.DrivetrainConstants.Pigeon2Id, Constants.swerveBus);
                break;
            default:
                flDriveMotor = new MotorIO();
                flAngleMotor = new MotorIO();
                flEncoder = new EncoderIO();

                frDriveMotor = new MotorIO();
                frAngleMotor = new MotorIO();
                frEncoder = new EncoderIO();

                blDriveMotor = new MotorIO();
                blAngleMotor = new MotorIO();
                blEncoder = new EncoderIO();

                brDriveMotor = new MotorIO();
                brAngleMotor = new MotorIO();
                brEncoder = new EncoderIO();

                gyro = new GyroIO();
                break;
        }
        // Create swerve subsystem
        SwerveModule fl = new SwerveModule(flDriveMotor, flAngleMotor, flEncoder, 0, TunerConstants.FrontLeft);
        SwerveModule fr = new SwerveModule(frDriveMotor, frAngleMotor, frEncoder, 1, TunerConstants.FrontRight);
        SwerveModule bl = new SwerveModule(blDriveMotor, blAngleMotor, blEncoder, 2, TunerConstants.BackLeft);
        SwerveModule br = new SwerveModule(brDriveMotor, brAngleMotor, brEncoder, 3, TunerConstants.BackRight);

        swerve = new Swerve(gyro, fl, fr, bl, br);

        if (Constants.currentMode == Mode.SIM) {
            // Initialize simulations for each component in sim mode
            new ArmSim(armMotor, armEncoder);
            new ElevatorSubsystemSim(elevatorLeftMotor, elevatorRightMotor, elevatorEncoder);
            new WristSim(wristMotor, wristEncoder);

            new SwerveModuleSim(flDriveMotor, flAngleMotor, flEncoder, TunerConstants.FrontLeft);
            new SwerveModuleSim(frDriveMotor, frAngleMotor, frEncoder, TunerConstants.FrontRight);
            new SwerveModuleSim(blDriveMotor, blAngleMotor, blEncoder, TunerConstants.BackLeft);
            new SwerveModuleSim(brDriveMotor, brAngleMotor, brEncoder, TunerConstants.BackRight);

            new GyroSim(gyro);
        }
    }

    private void initCommands() {
        armCommands = new ArmCommands(arm);
        elevatorCommands = new ElevatorCommands(elevator);
        wristCommands = new WristCommands(wrist);
        hangCommands = new HangCommands(hang);
        intakeCommands = new IntakeCommands(intake);
        swerveCommands = new SwerveCommands(swerve);
    }

    private void configureBindings() {
        /* ---- Main controller bindings ---- */

        // PID-based forward movement (CCW)
        controller.cross().and(() -> !Arm.Constants.manualArm.get()).onTrue(armCommands.setGoal(() -> 0.5));

        // Manual forward movement (CCW)
        controller
                .cross()
                .and(() -> Arm.Constants.manualArm.get())
                .onTrue(armCommands.setSpeed(() -> 0.2))
                .onFalse(armCommands.stop());

        // PID-based backward movement (CW)
        controller.circle().and(() -> !Arm.Constants.manualArm.get()).onTrue(armCommands.setGoal(() -> 2));

        // Manual backward movement (CW)
        controller
                .circle()
                .and(() -> Arm.Constants.manualArm.get())
                .onTrue(armCommands.setSpeed(() -> -0.2))
                .onFalse(armCommands.stop());

        // Elevator controls
        // XPID-based elevator up to middle position (L1 button)
        controller.L1().and(() -> !Elevator.Constants.manualElevator.get()).onTrue(elevatorCommands.goToMiddle());

        // Manual elevator up (L1 button in manual mode)
        controller
                .L1()
                .and(() -> Elevator.Constants.manualElevator.get())
                .onTrue(elevatorCommands.setSpeed(() -> 0.3))
                .onFalse(elevatorCommands.stop());

        // PID-based elevator down to bottom position (L2 button)
        controller.L2().and(() -> !Elevator.Constants.manualElevator.get()).onTrue(elevatorCommands.goToBottom());

        // Manual elevator down (L2 button in manual mode)
        controller
                .L2()
                .and(() -> Elevator.Constants.manualElevator.get())
                .onTrue(elevatorCommands.setSpeed(() -> -0.3))
                .onFalse(elevatorCommands.stop());

        // PID-based elevator to top position (R1 button)
        controller.R1().and(() -> !Elevator.Constants.manualElevator.get()).onTrue(elevatorCommands.goToTop());

        // Manual elevator up faster (R1 button in manual mode)
        controller
                .R1()
                .and(() -> Elevator.Constants.manualElevator.get())
                .onTrue(elevatorCommands.setSpeed(() -> 0.5))
                .onFalse(elevatorCommands.stop());

        // Wrist controls
        // PID-based wrist to straight position (triangle button)
        controller.triangle().and(() -> !Wrist.Constants.manualWrist.get()).onTrue(wristCommands.goToStraight());

        // Manual wrist up (triangle button in manual mode)
        controller
                .triangle()
                .and(() -> Wrist.Constants.manualWrist.get())
                .onTrue(wristCommands.setSpeed(() -> 0.2))
                .onFalse(wristCommands.stop());

        // PID-based wrist to down position (square button)
        controller.povDown().and(() -> !Wrist.Constants.manualWrist.get()).onTrue(wristCommands.goToDown());

        // Manual wrist down (square button in manual mode)
        controller
                .square()
                .and(() -> Wrist.Constants.manualWrist.get())
                .onTrue(wristCommands.setSpeed(() -> -0.2))
                .onFalse(wristCommands.stop());

        // PID-based wrist to stow position (R2 button)
        controller.povUp().and(() -> !Wrist.Constants.manualWrist.get()).onTrue(wristCommands.goToUp());

        // Manual wrist down faster (R2 button in manual mode)
        controller
                .R2()
                .and(() -> Wrist.Constants.manualWrist.get())
                .onTrue(wristCommands.setSpeed(() -> -0.3))
                .onFalse(wristCommands.stop());

        // Hang controls (for climbing at end of match)
        // Hang extend up at full speed (left stick up)
        controller.povLeft().onTrue(hangCommands.extendUp()).onFalse(hangCommands.stop());

        // Hang retract down at full speed (left stick down)
        controller.povRight().onTrue(hangCommands.retractDown()).onFalse(hangCommands.stop());

        // Hang controls (for climbing at end of match)
        // Hang extend up at full speed (left stick up)
        controller.L1().onTrue(intakeCommands.intakeFull()).onFalse(intakeCommands.stop());

        // Hang retract down at full speed (left stick down)
        controller.R1().onTrue(intakeCommands.outtakeFull()).onFalse(intakeCommands.stop());

        swerve.setDefaultCommand(swerveCommands.drive(
                () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX(), () -> true));
    }

    // Run selected subsystem at given duty cycle
    private void runSelectedTest(double dutyCycle) {
        if (testControllerChooser.get().equals("Arm")) {
            arm.setSpeed(dutyCycle);
        } else if (testControllerChooser.get().equals("Elevator")) {
            elevator.setSpeed(dutyCycle);
        } else if (testControllerChooser.get().equals("Wrist")) {
            wrist.setSpeed(dutyCycle);
        } else if (testControllerChooser.get().equals("Hang")) {
            hang.setSpeed(dutyCycle);
        } else if (testControllerChooser.get().equals("Intake")) {
            intake.setSpeed(dutyCycle);
        }
    }

    private void configureTestBindings() {
        /* ---- Test controller bindings ---- */
        testControllerChooser = new LoggedDashboardChooser<>("Test/Type");
        testControllerChooser.addOption("Arm", "Arm");
        testControllerChooser.addOption("Elevator", "Elevator");
        testControllerChooser.addOption("Wrist", "Wrist");
        testControllerChooser.addOption("Hang", "Hang");
        testControllerChooser.addOption("Intake", "Intake");

        testController
                .cross()
                .onTrue(Commands.runOnce(() -> runSelectedTest(0.2)))
                .onFalse(Commands.runOnce(() -> runSelectedTest(0)));

        testController
                .circle()
                .onTrue(Commands.runOnce(() -> runSelectedTest(-0.2)))
                .onFalse(Commands.runOnce(() -> runSelectedTest(0)));
    }

    private void configureSysId() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void periodic() {
        publisher.publish();
    }
}
