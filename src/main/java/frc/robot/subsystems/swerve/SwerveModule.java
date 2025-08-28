package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

public class SwerveModule {
    private MotorIO driveMotor;
    private MotorIO angleMotor;
    private EncoderIO angleEncoder;

    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private Alert driveMotorDisconnect;
    private Alert driveMotorHardwareFault;
    private Alert driveMotorOverheat;

    private Alert angleMotorDisconnect;
    private Alert angleMotorHardwareFault;
    private Alert angleMotorOverheat;

    private Alert encoderDisconnect;
    private Alert encoderHardwareFault;
    private Alert encoderMagnetFault;

    private int index;

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

        driveMotor.setBraking(true);
        driveMotor.setGains(constants.DriveMotorGains);
        driveMotor.setGearRatio(constants.DriveMotorGearRatio);
        driveMotor.setStatorCurrentLimit(constants.SlipCurrent);
        driveMotor.setInverted(constants.DriveMotorInverted);

        angleMotor.setBraking(true);
        angleMotor.setGains(constants.SteerMotorGains);
        angleMotor.connectCANcoder(constants.EncoderId, constants.SteerMotorGearRatio, 1);
        angleMotor.setContinuousWrap(true);
        angleMotor.setInverted(constants.SteerMotorInverted);
        angleMotor.setOffset(Units.rotationsToRadians(
                constants.EncoderOffset)); // Fix encoder zero position (convert from rotations to radians)

        angleEncoder.setInverted(constants.EncoderInverted);

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

        driveMotorDisconnect =
                new Alert("The drive motor on module " + modulePos + " is disconnected", AlertType.kError);
        driveMotorHardwareFault = new Alert(
                "The drive motor on module " + modulePos + " has encountered a hardware error", AlertType.kError);
        driveMotorOverheat =
                new Alert("The drive motor on module " + modulePos + " is overheating", AlertType.kWarning);

        angleMotorDisconnect =
                new Alert("The angle motor on module " + modulePos + " is disconnected", AlertType.kError);
        angleMotorHardwareFault = new Alert(
                "The angle motor on module " + modulePos + " has encountered a hardware error", AlertType.kError);
        angleMotorOverheat =
                new Alert("The angle motor on module " + modulePos + " is overheating", AlertType.kWarning);

        encoderDisconnect = new Alert("The encoder on module " + modulePos + " is disconnected", AlertType.kError);
        encoderHardwareFault =
                new Alert("The encoder on module " + modulePos + " has encountered a hardware fault", AlertType.kError);
        encoderMagnetFault =
                new Alert("The encoder on module " + modulePos + " has a non-functioning magnet", AlertType.kError);
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
        return new SwerveModulePosition(getPositionMeters(), Rotation2d.fromRadians(getAngle()));
    }

    // Get the module's current state: how fast it's going and which way it's pointing
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), Rotation2d.fromRadians(getAngle()));
    }

    // This runs every robot loop (about 50 times per second) to update sensors and check for problems
    public void periodic() {
        driveMotor.updateInputs();
        angleMotor.updateInputs();
        angleEncoder.updateInputs();

        Logger.processInputs("Drive/Module" + index + "/DriveMotor", driveMotor.getInputs());
        Logger.processInputs("Drive/Module" + index + "/AngleMotor", angleMotor.getInputs());
        Logger.processInputs("Drive/Module" + index + "/AngleEncoder", angleEncoder.getInputs());

        driveMotorDisconnect.set(!driveMotor.getInputs().connected);
        driveMotorHardwareFault.set(driveMotor.getInputs().hardwareFault);
        driveMotorOverheat.set(driveMotor.getInputs().tempFault);

        angleMotorDisconnect.set(!angleMotor.getInputs().connected);
        angleMotorHardwareFault.set(angleMotor.getInputs().hardwareFault);
        angleMotorOverheat.set(angleMotor.getInputs().tempFault);

        encoderDisconnect.set(!angleEncoder.getInputs().connected);
        encoderHardwareFault.set(angleEncoder.getInputs().hardwareFault);
        encoderMagnetFault.set(angleEncoder.getInputs().badMagnetFault);
    }
}
