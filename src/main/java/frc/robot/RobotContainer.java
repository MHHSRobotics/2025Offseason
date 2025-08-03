package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.ArmCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.io.EncoderIO;
import frc.robot.io.EncoderIOCANcoder;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon;
import frc.robot.io.MotorIO;
import frc.robot.io.MotorIOTalonFX;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleSim;
import frc.robot.subsystems.swerve.TunerConstants;

public class RobotContainer {
    private Arm arm;
    private Swerve swerve;

    private ArmCommands armCommands;
    private SwerveCommands swerveCommands;

    // Main drive controller
    private final CommandPS5Controller controller = new CommandPS5Controller(0);

    // Controller for SysId commands
    private final CommandPS5Controller sysIdController = new CommandPS5Controller(2);

    public RobotContainer() {
        // Initialize all the IO objects, subsystems, and mechanism simulators
        initSubsystems();
        armCommands = new ArmCommands(arm);
        swerveCommands = new SwerveCommands(swerve);

        // Add controller bindings
        configureBindings();

        // Add SysId bindings
        configureSysId();
    }

    private void initSubsystems() {
        // Create variables for all motors and encoders
        MotorIO armMotor;
        EncoderIO armEncoder;

        MotorIO flDriveMotor;
        MotorIO flAngleMotor;
        EncoderIO flEncoder;
        MotorIO frDriveMotor;
        MotorIO frAngleMotor;
        EncoderIO frEncoder;
        MotorIO blDriveMotor;
        MotorIO blAngleMotor;
        EncoderIO blEncoder;
        MotorIO brDriveMotor;
        MotorIO brAngleMotor;
        EncoderIO brEncoder;

        GyroIO gyro;
        switch (Constants.currentMode) {
            case REAL:
            case SIM:
                armMotor = new MotorIOTalonFX(Arm.Constants.motorId, Constants.defaultBus);
                armEncoder = new EncoderIOCANcoder(Arm.Constants.encoderId, Constants.defaultBus);

                flDriveMotor=new MotorIOTalonFX(TunerConstants.FrontLeft.DriveMotorId, Constants.swerveBus);
                flAngleMotor=new MotorIOTalonFX(TunerConstants.FrontLeft.SteerMotorId, Constants.swerveBus);
                flEncoder=new EncoderIOCANcoder(TunerConstants.FrontLeft.EncoderId, Constants.swerveBus);

                frDriveMotor=new MotorIOTalonFX(TunerConstants.FrontRight.DriveMotorId, Constants.swerveBus);
                frAngleMotor=new MotorIOTalonFX(TunerConstants.FrontRight.SteerMotorId, Constants.swerveBus);
                frEncoder=new EncoderIOCANcoder(TunerConstants.FrontRight.EncoderId, Constants.swerveBus);

                blDriveMotor=new MotorIOTalonFX(TunerConstants.BackLeft.DriveMotorId, Constants.swerveBus);
                blAngleMotor=new MotorIOTalonFX(TunerConstants.BackLeft.SteerMotorId, Constants.swerveBus);
                blEncoder=new EncoderIOCANcoder(TunerConstants.BackLeft.EncoderId, Constants.swerveBus);

                brDriveMotor=new MotorIOTalonFX(TunerConstants.BackRight.DriveMotorId, Constants.swerveBus);
                brAngleMotor=new MotorIOTalonFX(TunerConstants.BackRight.SteerMotorId, Constants.swerveBus);
                brEncoder=new EncoderIOCANcoder(TunerConstants.BackRight.EncoderId, Constants.swerveBus);

                gyro=new GyroIOPigeon(TunerConstants.DrivetrainConstants.Pigeon2Id, Constants.swerveBus);
                if(RobotBase.isReal()){
                    break;
                }
                // Sim code only here
                new ArmSim(armMotor,armEncoder);

                new SwerveModuleSim(flDriveMotor,flAngleMotor,flEncoder,TunerConstants.FrontLeft);
                new SwerveModuleSim(frDriveMotor,frAngleMotor,frEncoder,TunerConstants.FrontRight);
                new SwerveModuleSim(blDriveMotor,blAngleMotor,blEncoder,TunerConstants.BackLeft);
                new SwerveModuleSim(brDriveMotor,brAngleMotor,flEncoder,TunerConstants.BackRight);
                
                break;

            default:
                armMotor = new MotorIO();
                armEncoder = new EncoderIO();

                flDriveMotor=new MotorIO();
                flAngleMotor=new MotorIO();
                flEncoder=new EncoderIO();

                frDriveMotor=new MotorIO();
                frAngleMotor=new MotorIO();
                frEncoder=new EncoderIO();

                blDriveMotor=new MotorIO();
                blAngleMotor=new MotorIO();
                blEncoder=new EncoderIO();

                brDriveMotor=new MotorIO();
                brAngleMotor=new MotorIO();
                brEncoder=new EncoderIO();

                gyro=new GyroIO();
                break;
        }

        arm = new Arm(armMotor, armEncoder);

        SwerveModule fl=new SwerveModule(flDriveMotor, flAngleMotor, flEncoder, 0, TunerConstants.FrontLeft);
        SwerveModule fr=new SwerveModule(frDriveMotor, frAngleMotor, frEncoder, 1, TunerConstants.FrontRight);
        SwerveModule bl=new SwerveModule(blDriveMotor, blAngleMotor, blEncoder, 2, TunerConstants.BackLeft);
        SwerveModule br=new SwerveModule(brDriveMotor, brAngleMotor, brEncoder, 3, TunerConstants.BackRight);

        swerve=new Swerve(gyro, fl,fr,bl,br);
        
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

        swerve.setDefaultCommand(swerveCommands.drive(()->-controller.getLeftY(), ()->-controller.getLeftX(), ()->-controller.getRightX(), ()->true));
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
