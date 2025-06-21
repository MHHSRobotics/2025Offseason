package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedCANcoder;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;
import frc.robot.utils.RobotUtils;

import static edu.wpi.first.units.Units.Radians;

public class Arm extends SubsystemBase {

    public static class Constants {
        public static final int motorId = 22;
        public static final boolean motorInverted = false;
        public static final int encoderId = 1;
        public static final double encoderOffset = -1.052;

        public static final double rotorToSensorRatio = 25;
        public static final double sensorToMechanismRatio = 28 / 9.;

        public static final double kP = 200;
        public static final double kD = 0;

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

        public static final double gearRatio = rotorToSensorRatio * sensorToMechanismRatio;
        public static final double armLength = Math.sqrt(3 * moi / mass);
    }

    /** Motor, uses TalonFXIO for simulation support */
    private LoggedTalonFX motor;

    private LoggedCANcoder encoder;

    public Arm(TalonFXIO motor, CANcoderIO encoder) {
        this.motor = new LoggedTalonFX(motor, "Arm/Motor");
        this.encoder = new LoggedCANcoder(encoder, "Arm/Encoder");

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted =
                Constants.motorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        motorConfig.Feedback.FeedbackRemoteSensorID = Constants.encoderId;
        motorConfig.Feedback.RotorToSensorRatio = Constants.rotorToSensorRatio;
        motorConfig.Feedback.SensorToMechanismRatio = Constants.sensorToMechanismRatio;
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
        encoderConfig.MagnetSensor.withMagnetOffset(
                Radians.of(Constants.encoderOffset * Constants.sensorToMechanismRatio));

        motor.applyConfig(motorConfig);
        encoder.applyConfig(encoderConfig);
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
    }
}
