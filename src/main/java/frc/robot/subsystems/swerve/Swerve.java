package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.GyroIO;
import frc.robot.util.RobotUtils;

import static edu.wpi.first.units.Units.MetersPerSecond;

// Make the swerve drive move the robot in any direction and rotate at the same time.
// Uses a pose estimator to keep track of where the robot is on the field.
// Units:
// - Distances in meters (m)
// - Linear speeds in meters per second (m/s)
// - Angles in radians
// - Angular speeds in radians per second (rad/s)
public class Swerve extends SubsystemBase {
    public static class Constants {
        // How far the stick must move from center before the robot starts translating (0 to 1 range)
        public static final double moveDeadband = 0.1;

        // How far the stick must move from center before the robot starts turning (0 to 1 range)
        public static final double turnDeadband = 0.1;

        // Smart shortcut to make small moves easier: raise input to a power.
        // Example: stick = 0.5, movePow = 2 -> 0.5^2 = 0.25 (finer control near center)
        public static final double movePow = 2;

        // Same idea as movePow but for turning
        public static final double turnPow = 2;

        public static final LoggedNetworkBoolean swerveLocked =
                new LoggedNetworkBoolean("Swerve/Locked", true); // Toggle to enable braking when stopped

        public static final LoggedNetworkBoolean swerveDisabled = new LoggedNetworkBoolean(
                "Swerve/Disabled", false); // Toggle to completely disable all motors in the swerve subsystem
    }

    // Find out the robot heading from the gyro (real or simulated)
    private GyroIO gyro;

    // The four swerve modules, in order: front-left, front-right, back-left, back-right
    private SwerveModule[] modules = new SwerveModule[4];

    // Current robot heading (radians) used by the pose estimator
    private Rotation2d gyroAngle = new Rotation2d();

    // Math helper that converts chassis speeds (vx, vy, omega) into wheel angles and speeds
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    // Make the best guess of the robot's field position using wheel odometry and gyro (and vision if added later)
    private SwerveDrivePoseEstimator estimator;

    // On-screen drawing of the drive to show module directions and speeds
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // Base points for each module drawing
    private final LoggedMechanismRoot2d[] roots = new LoggedMechanismRoot2d[4];

    // Bars showing the speed (length) and direction (angle) of each module
    private final LoggedMechanismLigament2d[] speeds = new LoggedMechanismLigament2d[4];

    public Swerve(GyroIO gyro, SwerveModule fl, SwerveModule fr, SwerveModule bl, SwerveModule br) {
        this.gyro = gyro;
        this.modules = new SwerveModule[] {fl, fr, bl, br};

        // Tell the gyro what to call itself for alerts and where to log data
        gyro.setName("gyro");
        gyro.setPath("Swerve/Gyro");

        estimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, getModulePositions(), new Pose2d());
        // Set up the on-screen visualization for the four modules
        initializeMechs();
    }

    // Find out where the modules are mounted on the robot relative to the center (meters)
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    // Make a white line between two modules in the visualization so the robot outline is visible
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
        // Create roots at the four corners of the swerve bot drawing
        for (int i = 0; i < 4; i++) {
            LoggedMechanismRoot2d newRoot =
                    mech.getRoot("Root" + i, 1.5 + moduleTranslations[i].getX(), 1.5 + moduleTranslations[i].getY());
            roots[i] = newRoot;
            // The speed lines are added below after we draw the connections
        }
        // Connect each pair of swerve modules to outline the drivetrain
        connect(0, 1);
        connect(0, 2);
        connect(1, 3);
        connect(2, 3);
        for (int i = 0; i < 4; i++) {
            speeds[i] = roots[i].append(new LoggedMechanismLigament2d("Speed" + i, 0, 0, 5, new Color8Bit(Color.kRed)));
        }
    }

    // Make the visualization match the real module speeds and directions
    private void refreshVisualization() {
        SwerveModuleState[] states = getModuleStates();
        for (int i = 0; i < 4; i++) {
            // Scale length down so it fits nicely on screen
            speeds[i].setLength(states[i].speedMetersPerSecond / 10);
            speeds[i].setAngle(states[i].angle);
        }
    }

    // Find out the robot's current field position (x, y in meters, rotation in radians)
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    // Find out the robot's current heading (radians)
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    // Tell the pose estimator to reset to a known field position (meters, radians)
    public void setPose(Pose2d newPose) {
        estimator.resetPosition(gyroAngle, getModulePositions(), newPose);
    }

    // Find out each module's odometry: wheel angle (radians) and total distance driven (meters)
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    // Find out each wheel's change in position
    public SwerveModulePosition[] getModuleDeltas() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPositionDelta();
        }
        return states;
    }

    // Find out each module's current state: wheel angle (radians) and speed (m/s)
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    // Tell the robot how fast to move:
    // - dx, dy are strafe commands in X/Y where -1..1 maps to full speed (m/s)
    // - omega is turn command where -1..1 maps to full turn speed (rad/s)
    // - fieldRelative = true means the commands are relative to the field (forward = away from our driver station)
    //   and will auto-flip for alliance side using RobotUtils
    public void runSpeeds(double dx, double dy, double omega, boolean fieldRelative) {
        ChassisSpeeds speeds = new ChassisSpeeds(
                dx * getMaxLinearSpeedMetersPerSec(),
                dy * getMaxLinearSpeedMetersPerSec(),
                omega * getMaxAngularSpeedRadPerSec());
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotUtils.invertToAlliance(getRotation()));
        }
        runVelocity(speeds);
    }

    // Tell the modules to reach a target chassis speed: vx (m/s), vy (m/s), omega (rad/s)
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
    }

    // Tell the swerve to stop moving (vx = 0, vy = 0, omega = 0)
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    // Make the modules point in an X pattern so it's harder to push the robot.
    // The next call to runVelocity() will return the modules to normal control.
    public void lock() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    // Find out the robot's top speed (m/s) at 12 volts from characterization
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    // Drive base radius (meters): farthest distance from robot center to any module
    private static final double driveBaseRadius = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // Find out the robot's top turn rate (rad/s) based on top linear speed and radius
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / driveBaseRadius;
    }

    // Add a vision measurement with the given pose, timestamp, and standard deviations
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        estimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // This runs every robot loop (~50 times per second)
        // 1) Run each module's periodic to update sensors and control, and brake and disable if necessary
        for (SwerveModule module : modules) {
            module.setLocked(Constants.swerveLocked.get());
            module.setDisabled(Constants.swerveDisabled.get());
            module.periodic();
        }

        // 2) Update the gyro inputs (logging and alerts happen automatically)
        gyro.update();

        if (gyro.getInputs().connected) {
            // If gyro is connected, read the angle
            gyroAngle = Rotation2d.fromRadians(gyro.getInputs().yawPositionRad);
        } else {
            // If gyro is disconnected, like in sim, get module deltas and use odometry to figure out the change in
            // angle
            Twist2d twist = kinematics.toTwist2d(getModuleDeltas());
            gyroAngle = gyroAngle.plus(Rotation2d.fromRadians(twist.dtheta));
        }
        // 3) Feed odometry to the pose estimator (time, heading, and wheel distances)
        estimator.updateWithTime(RobotController.getFPGATime(), gyroAngle, getModulePositions());

        // 4) Update the module speed/direction drawing
        refreshVisualization();

        Logger.recordOutput("Swerve/Visualization", mech);
    }
}
