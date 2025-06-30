package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedCANcoder;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;

public class Arm extends SubsystemBase {

    public static class Constants {
        // All radians are mechanism radians

        public static final int motorId = 22;
        public static final boolean motorInverted = false;
        public static final int encoderId = 1;
        public static final double encoderOffset = -1.052; // radians

        public static final double gearRatio = 700 / 9.; // ratio of motor rotations to mechanism rotations
        public static final double encoderRatio = 28 / 9.; // ratio of encoder rotations to mechanism rotations

        public static final double kP = 200; // amps per radian, current scales with distance to setpoint
        public static final double kD = 20; // amps per radian per sec

        public static final double kS = 0; // amps, the current needed to overcome static friction
        public static final double kG = 20; // amps, the current needed to overcome gravity when the arm is horizontal
        public static final double kV =
                0; // amps per radian per sec, current needed to overcome linear friction (scales with velocity)
        public static final double kA =
                0; // amps per radian per sec^2, current needed to overcome quadratic friction (scales with
        // acceleration)

        public static final double maxVelocity = 100; // radians per sec, sets the max velocity MotionMagic will use
        public static final double maxAccel = 200; // radians per sec^2, sets the max acceleration MotionMagic will use

        public static final double moi = 4.8944; // kg m^2, how hard it is to rotate the arm
        public static final double mass = 10; // kg

        public static final double minAngle = Units.degreesToRadians(-45);
        public static final double maxAngle = Units.degreesToRadians(140);
        public static final double startAngle = Units.degreesToRadians(90); // start angle for the simulated arm

        public static final double rotorToSensorRatio =
                gearRatio / encoderRatio; // ratio of motor rotations to encoder rotations
        public static final double armLength =
                Math.sqrt(3 * moi / mass); // math to make WPILib simulate the arm correctly

        public static final LoggedNetworkBoolean manualArm =
                new LoggedNetworkBoolean("Arm/Manual Arm", false); // toggles whether the arm is in manual control
    }

    // Motor, uses LoggedTalonFX to automatically handle simulation
    private LoggedTalonFX motor;

    // LoggedCANcoder automatically handles simulation
    private LoggedCANcoder encoder;

    // Mechanism visualization
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // The fixed end of the arm visualization
    private final LoggedMechanismRoot2d root = mech.getRoot("ArmRoot", 1, 1.5);

    // The arm visualization
    private final LoggedMechanismLigament2d arm =
            root.append(new LoggedMechanismLigament2d("Arm", 1.0, 0, 6, new Color8Bit(Color.kRed)));

    // Visualization of the target position of the arm
    private final LoggedMechanismLigament2d goalArm =
            root.append(new LoggedMechanismLigament2d("GoalArm", 1.0, 0, 6, new Color8Bit(Color.kYellow)));

    // Fixed end of the proportional visualization
    private final LoggedMechanismRoot2d pRoot = mech.getRoot("PRoot", 2.5, 2);

    // Fixed end of the derivative visualization
    private final LoggedMechanismRoot2d dRoot = mech.getRoot("DRoot", 2.6, 2);

    // Fixed end of the feedforward visualization
    private final LoggedMechanismRoot2d fRoot = mech.getRoot("FRoot", 2.7, 2);

    // Proportional visualization
    private final LoggedMechanismLigament2d pAmount =
            pRoot.append(new LoggedMechanismLigament2d("PAmount", 1.0, 90, 6, new Color8Bit(Color.kBlue)));

    // Derivative visualization
    private final LoggedMechanismLigament2d dAmount =
            dRoot.append(new LoggedMechanismLigament2d("DAmount", 1.0, 90, 6, new Color8Bit(Color.kGreen)));

    // Feedforward visualization
    private final LoggedMechanismLigament2d fAmount =
            fRoot.append(new LoggedMechanismLigament2d("FAmount", 1.0, 90, 6, new Color8Bit(Color.kWhite)));

    public Arm(TalonFXIO motorIO, CANcoderIO encoderIO) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // Inverts the motor depending on Constants.motorInverted
        motorConfig.MotorOutput.Inverted =
                Constants.motorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // Fuses the motor to the encoder. Note: don't put the offset here, it's already handled by CANcoderIOBase
        motorConfig.Feedback.FeedbackRemoteSensorID = Constants.encoderId;
        motorConfig.Feedback.RotorToSensorRatio = Constants.rotorToSensorRatio;
        motorConfig.Feedback.SensorToMechanismRatio = Constants.encoderRatio;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // PID setup
        motorConfig.Slot0.kP = Constants.kP;
        motorConfig.Slot0.kD = Constants.kD;

        // Sets feedforward type to arm
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Feedforward contants
        motorConfig.Slot0.kS = Constants.kS;
        motorConfig.Slot0.kG = Constants.kG;
        motorConfig.Slot0.kV = Constants.kV;
        motorConfig.Slot0.kA = Constants.kA;

        // Motion magic constants
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.maxVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = Constants.maxAccel;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        // Create logged motors and encoders from the configs
        motor = new LoggedTalonFX(motorIO, "Arm/Motor", motorConfig);
        encoder = new LoggedCANcoder(encoderIO, "Arm/Encoder", encoderConfig);
    }

    // Sets the speed of the arm. Speed is from -1 (full backward) to 1 (full forward)
    public void setSpeed(double value) {
        motor.setSpeed(value);
    }

    // Sets the goal of the arm in radians
    public void setGoal(double pos) {
        motor.setGoal(MathUtil.clamp(pos, Constants.minAngle, Constants.maxAngle));
    }

    // Returns the goal of the arm
    public double getGoal() {
        return motor.getGoal();
    }

    @Override
    public void periodic() {
        // Call periodic methods
        motor.periodic();
        encoder.periodic();

        // Set angles of the visualization arm and goal arm
        arm.setAngle(Rotation2d.fromRadians(motor.getPosition()));

        if (motor.getInputs().controlMode.equals("MotionMagicTorqueCurrentFOC")) {
            // If motor is currently in PID mode, show all the lines
            goalArm.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);

            // Set the angles/lengths of the lines
            goalArm.setAngle(Rotation2d.fromRadians(motor.getGoal()));
            pAmount.setLength(motor.getInputs().propOutput / 100);
            dAmount.setLength(motor.getInputs().derivOutput / 100);
            fAmount.setLength(motor.getInputs().feedforward / 100);
        } else {
            // Make all the lines invisible
            goalArm.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
        }

        // Log the mechanism
        Logger.recordOutput("Arm/Mech", mech);
    }
}
