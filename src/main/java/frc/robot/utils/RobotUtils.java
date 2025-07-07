package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

// Utility functions used it multiple places
public class RobotUtils {
    /**
     * Checks if the robot is on the red alliance.
     *
     * @return True if the robot is on the red alliance, false otherwise.
     */
    public static boolean onRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    // Rotates a Pose2d 180 degrees around the center of the field
    public static Pose2d invert(Pose2d pose) {
        return new Pose2d(
                Field.fieldLength - pose.getX(),
                Field.fieldWidth - pose.getY(),
                pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    // Inverts a Pose2d if on the red alliance
    public static Pose2d invertToAlliance(Pose2d pose) {
        if (onRedAlliance()) {
            return invert(pose);
        } else {
            return pose;
        }
    }

    // Rotates a Pose3d 180 degrees around the center of the field
    public static Pose3d invert(Pose3d pose) {
        return new Pose3d(
                Field.fieldLength - pose.getX(),
                Field.fieldWidth - pose.getY(),
                pose.getZ(),
                pose.getRotation().rotateBy(new Rotation3d(Rotation2d.k180deg)));
    }

    // Inverts a Pose3d if on the red alliance
    public static Pose3d invertToAlliance(Pose3d pose) {
        if (onRedAlliance()) {
            return invert(pose);
        } else {
            return pose;
        }
    }

    // Rotates a Translation2d 180 degrees around the center of the field
    public static Translation2d invert(Translation2d trans) {
        return new Translation2d(Field.fieldLength - trans.getX(), Field.fieldWidth - trans.getY());
    }

    // Inverts a Translation2d if on the red alliance
    public static Translation2d invertToAlliance(Translation2d trans) {
        if (onRedAlliance()) {
            return invert(trans);
        } else {
            return trans;
        }
    }

    // Rotates a Translation3d 180 degrees around the center of the field
    public static Translation3d invert(Translation3d trans) {
        return new Translation3d(Field.fieldLength - trans.getX(), Field.fieldWidth - trans.getY(), trans.getZ());
    }

    // Inverts a Translation3d if on the red alliance
    public static Translation3d invertToAlliance(Translation3d trans) {
        if (onRedAlliance()) {
            return invert(trans);
        } else {
            return trans;
        }
    }
}
