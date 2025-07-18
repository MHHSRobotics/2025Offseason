package frc.robot.subsystems.swerve;

import java.io.File;
import java.io.FileReader;

import edu.wpi.first.wpilibj.Filesystem;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class Swerve {
    public static class Constants {
        private static JSONObject constants;

        static {
            try {
                constants = (JSONObject) new JSONParser()
                        .parse(new FileReader(new File(Filesystem.getDeployDirectory(), "swerve.json")));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        public static final double driveInertia = (double) constants.get("drive_inertia");
        public static final double steerInertia = (double) constants.get("steer_inertia");
        public static final double driveFrictionVoltage = (double) constants.get("drive_friction_voltage");
        public static final double steerFrictionVoltage = (double) constants.get("steer_friction_voltage");
        public static final double driveRatio = (double) constants.get("drive_ratio");
        public static final double steerRatio = (double) constants.get("steer_ratio");
        public static final double wheelRadius = (double) constants.get("wheel_radius");
        public static final double slipCurrent = (double) constants.get("slip_current");
        public static final double maxSpeed = (double) constants.get("max_speed");
        public static final double maxAngularSpeed = (double) constants.get("max_angular_speed");
        public static final double currentLimit = (double) constants.get("current_limit");

        private static final JSONObject frontLeft = (JSONObject) constants.get("front_left");
        public static final double frontLeftX = (double) frontLeft.get("x");
        public static final double frontLeftY = (double) frontLeft.get("y");
        private static final JSONObject frontLeftDrive = (JSONObject) frontLeft.get("drive_motor");
        public static final int frontLeftDriveId = (int) (long) frontLeftDrive.get("id");
        public static final boolean frontLeftDriveInverted = (boolean) frontLeftDrive.get("inverted");
        private static final JSONObject frontLeftAngle = (JSONObject) frontLeft.get("angle_motor");
        public static final int frontLeftAngleId = (int) (long) frontLeftAngle.get("id");
        public static final boolean frontLeftAngleInverted = (boolean) frontLeftAngle.get("inverted");
        private static final JSONObject frontLeftEncoder = (JSONObject) frontLeft.get("encoder");
        public static final int frontLeftEncoderId = (int) (long) frontLeftEncoder.get("id");
        public static final boolean frontLeftEncoderInverted = (boolean) frontLeftEncoder.get("inverted");
        public static final double frontLeftEncoderOffset = (double) frontLeftEncoder.get("offset");

        private static final JSONObject frontRight = (JSONObject) constants.get("front_right");
        public static final double frontRightX = (double) frontRight.get("x");
        public static final double frontRightY = (double) frontRight.get("y");
        private static final JSONObject frontRightDrive = (JSONObject) frontRight.get("drive_motor");
        public static final int frontRightDriveId = (int) (long) frontRightDrive.get("id");
        public static final boolean frontRightDriveInverted = (boolean) frontRightDrive.get("inverted");
        private static final JSONObject frontRightAngle = (JSONObject) frontRight.get("angle_motor");
        public static final int frontRightAngleId = (int) (long) frontRightAngle.get("id");
        public static final boolean frontRightAngleInverted = (boolean) frontRightAngle.get("inverted");
        private static final JSONObject frontRightEncoder = (JSONObject) frontRight.get("encoder");
        public static final int frontRightEncoderId = (int) (long) frontRightEncoder.get("id");
        public static final boolean frontRightEncoderInverted = (boolean) frontRightEncoder.get("inverted");
        public static final double frontRightEncoderOffset = (double) frontRightEncoder.get("offset");

        private static final JSONObject backLeft = (JSONObject) constants.get("back_left");
        public static final double backLeftX = (double) backLeft.get("x");
        public static final double backLeftY = (double) backLeft.get("y");
        private static final JSONObject backLeftDrive = (JSONObject) backLeft.get("drive_motor");
        public static final int backLeftDriveId = (int) (long) backLeftDrive.get("id");
        public static final boolean backLeftDriveInverted = (boolean) backLeftDrive.get("inverted");
        private static final JSONObject backLeftAngle = (JSONObject) backLeft.get("angle_motor");
        public static final int backLeftAngleId = (int) (long) backLeftAngle.get("id");
        public static final boolean backLeftAngleInverted = (boolean) backLeftAngle.get("inverted");
        private static final JSONObject backLeftEncoder = (JSONObject) backLeft.get("encoder");
        public static final int backLeftEncoderId = (int) (long) backLeftEncoder.get("id");
        public static final boolean backLeftEncoderInverted = (boolean) backLeftEncoder.get("inverted");
        public static final double backLeftEncoderOffset = (double) backLeftEncoder.get("offset");

        private static final JSONObject backRight = (JSONObject) constants.get("back_right");
        public static final double backRightX = (double) backRight.get("x");
        public static final double backRightY = (double) backRight.get("y");
        private static final JSONObject backRightDrive = (JSONObject) backRight.get("drive_motor");
        public static final int backRightDriveId = (int) (long) backRightDrive.get("id");
        public static final boolean backRightDriveInverted = (boolean) backRightDrive.get("inverted");
        private static final JSONObject backRightAngle = (JSONObject) backRight.get("angle_motor");
        public static final int backRightAngleId = (int) (long) backRightAngle.get("id");
        public static final boolean backRightAngleInverted = (boolean) backRightAngle.get("inverted");
        private static final JSONObject backRightEncoder = (JSONObject) backRight.get("encoder");
        public static final int backRightEncoderId = (int) (long) backRightEncoder.get("id");
        public static final boolean backRightEncoderInverted = (boolean) backRightEncoder.get("inverted");
        public static final double backRightEncoderOffset = (double) backRightEncoder.get("offset");
    }

    private SwerveModule[] modules = new SwerveModule[4];

    public Swerve(SwerveModule[] modules) {
        this.modules = modules;
    }
}
