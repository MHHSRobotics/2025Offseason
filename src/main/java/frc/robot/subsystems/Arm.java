package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.CANcoderIO;
import frc.robot.io.TalonFXIO;
import frc.robot.utils.RobotUtils;

public class Arm extends SubsystemBase{
    public static final int motorId=22;
    public static final boolean motorInverted=false;
    public static final int encoderId=1;
    public static final double encoderOffset=-1.052;

    public static final double rotorToSensorRatio=25;
    public static final double sensorToMechanismRatio=28/9.;

    public static final double kP=5;
    public static final double kD=1;

    public static final double kS = 0.135;
    public static final double kG = 1.5;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double maxVelocity = 10;
    public static final double maxAccel = 2;

    public static final double moi=4.8944;

    public static final double minAngle = Units.degreesToRadians(-45);
    public static final double maxAngle = Units.degreesToRadians(140);

    /** Motor, uses TalonFXIO for simulation support */
    private TalonFXIO motor;

    private CANcoderIO encoder;

    public Arm(TalonFXIO motor,CANcoderIO encoder){
        this.motor=motor;
        this.encoder=encoder;

        TalonFXConfiguration config=new TalonFXConfiguration();

        config.MotorOutput.Inverted=motorInverted?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive;

        config.Feedback.FeedbackRemoteSensorID=encoderId;
        config.Feedback.FeedbackRotorOffset=encoderOffset;
        config.Feedback.RotorToSensorRatio=rotorToSensorRatio;
        config.Feedback.SensorToMechanismRatio=sensorToMechanismRatio;
        config.Feedback.FeedbackSensorSource=FeedbackSensorSourceValue.FusedCANcoder;

        config.Slot0.GravityType=GravityTypeValue.Arm_Cosine;
        config.Slot0.kP=kP;
        config.Slot0.kD=kD;
        config.Slot0.kS=kS;
        config.Slot0.kG=kG;
        config.Slot0.kV=kV;
        config.Slot0.kA=kA;

        config.MotionMagic.MotionMagicCruiseVelocity=maxVelocity;
        config.MotionMagic.MotionMagicAcceleration=maxAccel;
        
        CANcoderConfiguration config2=new CANcoderConfiguration();
        
        motor.applyConfig(config);
        encoder.applyConfig(config2);
    }

    public void set(double value){
        motor.setSpeed(value);
    }

    public void setGoal(double pos){
        motor.setGoal(RobotUtils.clamp(pos,minAngle,maxAngle));
    }

    public double getGoal(){
        return motor.getInputs().setpoint;
    }

    @Override
    public void periodic(){
        motor.updateInputs();
        encoder.updateInputs();
        Logger.processInputs("Arm/Motor", motor.getInputs());
        Logger.processInputs("Arm/Encoder", encoder.getInputs());
    }
}
