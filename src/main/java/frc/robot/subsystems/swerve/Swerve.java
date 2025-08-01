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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.io.GyroIO;

public class Swerve extends SubsystemBase {
    public static class Constants {}

    private GyroIO gyro;
    private SwerveModule[] modules = new SwerveModule[4];

    private Rotation2d gyroAngle = new Rotation2d();

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private SwerveDrivePoseEstimator estimator =
            new SwerveDrivePoseEstimator(kinematics, gyroAngle, getWheelPositions(), new Pose2d());

    private Alert gyroDisconnect=new Alert("The gyro is disconnected",AlertType.kError);
    private Alert gyroHardwareFault=new Alert("The gyro has encountered a hardware fault",AlertType.kError);

    public Swerve(GyroIO gyro, SwerveModule fl, SwerveModule fr, SwerveModule bl, SwerveModule br) {
        this.gyro = gyro;
        this.modules = new SwerveModule[] {fl, fr, bl, br};
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void setPose(Pose2d newPose) {
        estimator.resetPosition(gyroAngle, getWheelPositions(), newPose);
    }

    public SwerveModulePosition[] getWheelPositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.periodic();
        }
        gyro.updateInputs();
        Logger.processInputs("Drive/Gyro", gyro.getInputs());
        gyroDisconnect.set(!gyro.getInputs().connected);
        gyroHardwareFault.set(gyro.getInputs().hardwareFault);
        estimator.updateWithTime(RobotController.getFPGATime(), gyroAngle, getWheelPositions());
    }
}
