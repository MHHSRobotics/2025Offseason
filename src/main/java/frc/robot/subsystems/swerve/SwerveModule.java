package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class SwerveModule {
    private MotorIO driveMotor;
    private MotorIO angleMotor;
    private EncoderIO angleEncoder;

    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private int index;

    private SwerveModulePosition lastPosition = new SwerveModulePosition();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    public SwerveModule(
            MotorIO driveMotorIO,
            MotorIO angleMotorIO,
            EncoderIO angleEncoderIO,
            int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        this.index = index;

        driveMotor = driveMotorIO;
        angleMotor = angleMotorIO;
        angleEncoder = angleEncoderIO;

        String modulePos = "[UNKNOWN]";
        switch (index) {
            case 0:
                modulePos = "FL";
                break;
            case 1:
                modulePos = "FR";
                break;
            case 2:
                modulePos = "BL";
                break;
            case 3:
                modulePos = "BR";
                break;
        }

        // Tell the motors what to call themselves for alerts and where to log data
        driveMotor.setName(modulePos + " drive");
        driveMotor.setPath("Drive/Module" + index + "/DriveMotor");
        angleMotor.setName(modulePos + " angle");
        angleMotor.setPath("Drive/Module" + index + "/AngleMotor");

        driveMotor.setBraking(true);
        driveMotor.setGains(constants.DriveMotorGains);
        driveMotor.setGearRatio(constants.DriveMotorGearRatio);
        driveMotor.setStatorCurrentLimit(constants.SlipCurrent);
        driveMotor.setInverted(constants.DriveMotorInverted);

        // Tell the encoder what to call itself for alerts and where to log data
        angleEncoder.setName(modulePos + " encoder");
        angleEncoder.setPath("Drive/Module" + index + "/AngleEncoder");
        angleEncoder.setInverted(constants.EncoderInverted);

        angleMotor.setBraking(true);
        angleMotor.setGains(constants.SteerMotorGains);
        angleMotor.connectEncoder(angleEncoder, constants.SteerMotorGearRatio);
        angleMotor.setContinuousWrap(true);
        angleMotor.setInverted(constants.SteerMotorInverted);
        angleMotor.setOffset(Units.rotationsToRadians(
                -constants.EncoderOffset)); // Fix encoder zero position (convert from rotations to radians)
    }

    // Sets whether the drive and angle motors should brake
    public void setLocked(boolean locked) {
        driveMotor.setBraking(locked);
        angleMotor.setBraking(locked);
    }

    // Sets whether the swerve module is disabled
    public void setDisabled(boolean disabled) {
        driveMotor.setDisabled(disabled);
        angleMotor.setDisabled(disabled);
    }

    // Tell the drive motor how much power to use (voltage in volts, like 12V battery)
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    // Tell the steering motor how much power to use (voltage in volts)
    public void setAngleVoltage(double voltage) {
        angleMotor.setVoltage(voltage);
    }

    // Tell the wheel how fast to spin (speed in radians per second)
    public void setDriveVelocity(double radPerSec) {
        driveMotor.setVelocityWithVoltage(radPerSec);
    }

    // Tell the wheel which direction to point (angle in radians, like 0 = forward)
    public void setAnglePosition(double position) {
        angleMotor.setGoalWithVoltage(position);
    }

    // Find out which direction the wheel is currently pointing (angle in radians)
    public double getAngle() {
        return angleMotor.getInputs().position;
    }

    // Find out how far the robot has driven (distance in meters)
    public double getPositionMeters() {
        return getWheelPosition() * constants.WheelRadius;
    }

    // Find out how fast the robot is moving (speed in meters per second)
    public double getVelocityMetersPerSec() {
        return driveMotor.getInputs().velocity * constants.WheelRadius;
    }

    // Find out how much the wheel has rotated (angle in radians)
    public double getWheelPosition() {
        return driveMotor.getInputs().position;
    }

    // Make the swerve module go a certain speed and direction (state has speed in m/s and angle in radians)
    public void runSetpoint(SwerveModuleState state) {
        // Smart optimization: instead of turning 180° and going forward, just go backward instead
        state.optimize(Rotation2d.fromRadians(getAngle()));

        // Slow down the wheel when it's still turning to the right angle (makes driving smoother)
        state.cosineScale(Rotation2d.fromRadians(getAngle()));

        // Convert robot speed (m/s) to wheel spin speed (rad/s) using wheel size
        setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        setAnglePosition(state.angle.getRadians());
    }

    // Special test mode for measuring how the robot moves (used by SysId tool)
    public void runCharacterization(double volts) {
        setDriveVoltage(volts); // Apply test voltage
        setAnglePosition(0); // Keep wheel pointing straight forward
    }

    // Turn off all motors in this swerve module
    public void stop() {
        setDriveVoltage(0);
        setAngleVoltage(0);
    }

    // Get the module's current position: how far it's driven and which way it's pointing
    public SwerveModulePosition getPosition() {
        return currentPosition;
    }

    // Get the position of the swerve module in the last tick
    public SwerveModulePosition getLastPosition() {
        return lastPosition;
    }

    // Gets the change in position in the last tick
    public SwerveModulePosition getPositionDelta() {
        return new SwerveModulePosition(
                currentPosition.distanceMeters - lastPosition.distanceMeters, currentPosition.angle);
    }

    // Get the module's current state: how fast it's going and which way it's pointing
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), Rotation2d.fromRadians(getAngle()));
    }

    // This runs every robot loop (about 50 times per second) to update sensors and check for problems
    public void periodic() {
        // All updates handle logging and alerts automatically
        driveMotor.update();
        angleMotor.update();
        angleEncoder.update();

        // Update last position
        lastPosition = currentPosition;

        // Update current position
        currentPosition = new SwerveModulePosition(getPositionMeters(), Rotation2d.fromRadians(getAngle()));
    }
}
