package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import org.littletonrobotics.junction.Logger;

import frc.robot.io.CANcoderIO;
import frc.robot.io.CANcoderIOInputsAutoLogged;
import frc.robot.io.TalonFXIO;
import frc.robot.io.TalonFXIOInputsAutoLogged;
import frc.robot.utils.RobotUtils;

import static edu.wpi.first.units.Units.Radians;

public class Arm extends SubsystemBase {
    public static final int motorId = 22;
    public static final boolean motorInverted = false;
    public static final int encoderId = 1;
    public static final double encoderOffset = -1.052;

    public static final double rotorToSensorRatio = 25;
    public static final double sensorToMechanismRatio = 28 / 9.;

    public static final double kP = 5;
    public static final double kD = 1;

    public static final double kS = 0.135;
    public static final double kG = 1.5;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double maxVelocity = 10;
    public static final double maxAccel = 2;

    public static final double moi = 4.8944;
    public static final double mass = 10;

    public static final double minAngle = Units.degreesToRadians(-45);
    public static final double maxAngle = Units.degreesToRadians(140);

    /** Motor, uses TalonFXIO for simulation support */
    private TalonFXIO motor;

    private TalonFXIOInputsAutoLogged motorInputs = new TalonFXIOInputsAutoLogged();

    private CANcoderIO encoder;
    private CANcoderIOInputsAutoLogged encoderInputs = new CANcoderIOInputsAutoLogged();

    public Arm(TalonFXIO motor, CANcoderIO encoder) {
        this.motor = motor;
        this.encoder = encoder;

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted =
                motorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        // motorConfig.Feedback.FeedbackRotorOffset = encoderOffset;
        motorConfig.Feedback.RotorToSensorRatio = rotorToSensorRatio;
        motorConfig.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kD = kD;
        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kG = kG;
        motorConfig.Slot0.kV = kV;
        motorConfig.Slot0.kA = kA;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = maxAccel;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withMagnetOffset(Radians.of(encoderOffset));

        motor.applyConfig(motorConfig);
        encoder.applyConfig(encoderConfig);
    }

    public void setSpeed(double value) {
        motor.setSpeed(value);
    }

    public void setGoal(double pos) {
        motor.setGoal(RobotUtils.clamp(pos, minAngle, maxAngle));
    }

    public double getGoal() {
        return motorInputs.setpoint;
    }

    @Override
    public void periodic() {
        motor.updateInputs(motorInputs);
        encoder.updateInputs(encoderInputs);
        Logger.processInputs("Arm/Motor", motorInputs);
        Logger.processInputs("Arm/Encoder", encoderInputs);
    }
}
