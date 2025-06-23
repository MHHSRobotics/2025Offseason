package frc.robot.subsystems;

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

import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedCANcoder;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;
import frc.robot.utils.RobotUtils;

public class Arm extends SubsystemBase {

    public static class Constants {
        public static final int motorId = 22;
        public static final boolean motorInverted = false;
        public static final int encoderId = 1;
        public static final double encoderOffset = -1.052;

        public static final double gearRatio = 700 / 9.;
        public static final double encoderRatio = 28 / 9.;

        public static final double kP = 50;
        public static final double kD = 200;

        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double maxVelocity = 100;
        public static final double maxAccel = 200;

        public static final double moi = 4.8944;
        public static final double mass = 10;

        public static final double minAngle = Units.degreesToRadians(-45);
        public static final double maxAngle = Units.degreesToRadians(140);
        public static final double startAngle = Units.degreesToRadians(90);

        public static final double rotorToSensorRatio = gearRatio / encoderRatio;
        public static final double armLength = Math.sqrt(3 * moi / mass);
    }

    /** Motor, uses TalonFXIO for simulation support */
    private LoggedTalonFX motor;

    private LoggedCANcoder encoder;

    // Mechanism visualization
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    private final LoggedMechanismRoot2d root = mech.getRoot("ArmRoot", 1.5, 0.5);
    private final LoggedMechanismLigament2d arm =
            root.append(new LoggedMechanismLigament2d("Arm", 1.0, 0, 6, new Color8Bit(Color.kRed)));
    private final LoggedMechanismLigament2d goalArm =
            root.append(new LoggedMechanismLigament2d("GoalArm", 1.0, 0, 6, new Color8Bit(Color.kYellow)));

    public Arm(TalonFXIO motorIO, CANcoderIO encoderIO) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted =
                Constants.motorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        motorConfig.Feedback.FeedbackRemoteSensorID = Constants.encoderId;
        motorConfig.Feedback.RotorToSensorRatio = Constants.rotorToSensorRatio;
        motorConfig.Feedback.SensorToMechanismRatio = Constants.encoderRatio;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.kP = Constants.kP;
        motorConfig.Slot0.kD = Constants.kD;
        motorConfig.Slot0.kS = Constants.kS;
        motorConfig.Slot0.kG = Constants.kG;
        motorConfig.Slot0.kV = Constants.kV;
        motorConfig.Slot0.kA = Constants.kA;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.maxVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = Constants.maxAccel;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = Constants.encoderOffset;

        motor = new LoggedTalonFX(motorIO, "Arm/Motor", motorConfig);
        encoder = new LoggedCANcoder(encoderIO, "Arm/Encoder", encoderConfig);
    }

    public void setSpeed(double value) {
        motor.setSpeed(value);
    }

    public void setGoal(double pos) {
        motor.setGoal(RobotUtils.clamp(pos, Constants.minAngle, Constants.maxAngle));
    }

    public double getGoal() {
        return motor.getGoal();
    }

    @Override
    public void periodic() {
        motor.periodic();
        encoder.periodic();
        arm.setAngle(Rotation2d.fromRadians(motor.getPositionRad()));
        goalArm.setAngle(Rotation2d.fromRadians(motor.getGoal()));
        Logger.recordOutput("Arm/Mech", mech);
    }
}
