package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import frc.robot.io.GyroIO;
import frc.robot.util.RobotUtils;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class Swerve extends SubsystemBase {
    public static class Constants {
        // Deadband in the move controls, how far away the stick needs to be from the origin before the bot starts
        // moving
        public static final double moveDeadband = 0.1;

        // Deadband in the turn controls, how far away the stick needs to be from the origin before the bot starts
        // turning
        public static final double turnDeadband = 0.1;

        // Exponent applied to the move to make small adjustments easier. E.g. instead of moving 0.5 when the stick is
        // at halfway, the bot would move 0.5^2 = 0.25 with movePow 2.
        public static final double movePow = 2;

        // Same as movePow but for rotation
        public static final double turnPow = 2;
    }

    // IO object to interface with the gyro
    private GyroIO gyro;

    // An array of the four swerve modules, in order FL, FR, BL, BR
    private SwerveModule[] modules = new SwerveModule[4];

    // Current angle reported by the gyro
    private Rotation2d gyroAngle = new Rotation2d();

    // Kinematics object that simulates the swerve drive and has helper methods to make control easier
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    // This is the pose estimator for the swerve drive. It looks at odometry and vision measurements and gives a best
    // estimate for the current location of the bot.
    private SwerveDrivePoseEstimator estimator;

    // Gyro alerts for disconnect or hardware fault
    private Alert gyroDisconnect = new Alert("The gyro is disconnected", AlertType.kError);
    private Alert gyroHardwareFault = new Alert("The gyro has encountered a hardware fault", AlertType.kError);

    // Mechanism visualization
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // Roots at each of the four swerve modules
    private final LoggedMechanismRoot2d[] roots = new LoggedMechanismRoot2d[4];

    // Ligaments representing the speed and direction of each swerve module
    private final LoggedMechanismLigament2d[] speeds = new LoggedMechanismLigament2d[4];

    public Swerve(GyroIO gyro, SwerveModule fl, SwerveModule fr, SwerveModule bl, SwerveModule br) {
        this.gyro = gyro;
        this.modules = new SwerveModule[] {fl, fr, bl, br};

        estimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, getModulePositions(), new Pose2d());
        // Initialize the mechanism visualization
        initializeMechs();
    }

    // Gets an array of all the positions of the swerve modules relative to the center of the bot.
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    // Helper method to create a line connecting two swerve modules with the given indices in the visualization
    private void connect(int firstIndex, int secondIndex) {
        Translation2d[] moduleTranslations = getModuleTranslations();
        Translation2d diff = moduleTranslations[secondIndex].minus(moduleTranslations[firstIndex]);
        roots[firstIndex].append(new LoggedMechanismLigament2d(
                "Connection" + firstIndex + "" + secondIndex,
                diff.getNorm(),
                diff.getAngle().getDegrees(),
                2,
                new Color8Bit(Color.kWhite)));
    }

    private void initializeMechs() {
        Translation2d[] moduleTranslations = getModuleTranslations();
        // Create roots at the four corners of the swerve bot
        for (int i = 0; i < 4; i++) {
            LoggedMechanismRoot2d newRoot =
                    mech.getRoot("Root" + i, 1.5 + moduleTranslations[i].getX(), 1.5 + moduleTranslations[i].getY());
            roots[i] = newRoot;
            // Create the line representing speed of the module
            // speeds[i] = newRoot.append(new LoggedMechanismLigament2d("Speed" + i, 0, 0, 5, new
            // Color8Bit(Color.kRed)));
        }
        // Connect each pair of swerve modules
        connect(0, 1);
        connect(0, 2);
        connect(1, 3);
        connect(2, 3);
        for (int i = 0; i < 4; i++) {
            speeds[i] = roots[i].append(new LoggedMechanismLigament2d("Speed" + i, 0, 0, 5, new Color8Bit(Color.kRed)));
        }
    }

    // Refreshes the speed ligaments in the visualization
    private void refreshVisualization() {
        SwerveModuleState[] states = getModuleStates();
        for (int i = 0; i < 4; i++) {
            // System.out.println(states[i].speedMetersPerSecond);
            speeds[i].setLength(states[i].speedMetersPerSecond / 10);
            speeds[i].setAngle(states[i].angle);
        }
    }

    // Gets the current Pose2d of the drive as given by the SwerveDrivePoseEstimator
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    // Gets the current rotation of the drive as given by the SwerveDrivePoseEstimator.
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    // Set the current pose estimate to the given newPose
    public void setPose(Pose2d newPose) {
        estimator.resetPosition(gyroAngle, getModulePositions(), newPose);
    }

    // Gets the current positions of each of the swerve modules, which contain their angles and total distance traveled.
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    // Gets the current states of each of the swerve modules, which contain their angles and velocity.
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    // Runs the swerve module with the given dx, dy, and omega. If fieldRelative is true then its movement will be
    // relative to the current alliance's field.
    public void runSpeeds(double dx, double dy, double omega, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(
                dx * getMaxLinearSpeedMetersPerSec(),
                dy * getMaxLinearSpeedMetersPerSec(),
                omega * getMaxAngularSpeedRadPerSec());
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotUtils.invertToAlliance(getRotation()));
        }
        // System.out.println(dx + "," + dy + "," + omega);
        runVelocity(speeds);
    }

    // Runs the given ChassisSpeeds object. ChassisSpeeds represents movement in the x and y directions and rotation.
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
    }

    // Stops all movement in the swerve modules
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    // Locks the swerve modules into an X position to prevent the bot from being pushed. When runVelocity() is called
    // again the modules will return to their regular positions.
    public void lock() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    // Returns the maximum speed of the robot in meters/s
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    // The drive base radius, which is the maximum distance from the center of the robot to a swerve module
    private static final double driveBaseRadius = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // Returns the maximum angular speed of the robot in rad/s
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / driveBaseRadius;
    }

    @Override
    public void periodic() {
        // Call periodic for each of the swerve modules
        for (SwerveModule module : modules) {
            module.periodic();
        }

        // Update and log the gyro inputs
        gyro.updateInputs();
        Logger.processInputs("Drive/Gyro", gyro.getInputs());

        // Update the gyro alerts
        gyroDisconnect.set(!gyro.getInputs().connected);
        gyroHardwareFault.set(gyro.getInputs().hardwareFault);

        // Pass odometry data into the SwerveDrivePoseEstimator
        estimator.updateWithTime(RobotController.getFPGATime(), gyroAngle, getModulePositions());

        // Refresh the speed ligaments in the visualization
        refreshVisualization();

        Logger.recordOutput("Drive/Mech", mech);
    }
}
