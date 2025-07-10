package frc.robot.subsystems;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedCANcoder;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;

public class Catvator extends SubsystemBase{
        public static class Constants{
            public static final int leftMotorId = 20;
            public static final int rightMotorId = 21;

            public static final boolean inverted = false;

            // Encoder ID
            public static final int encoderId = 2;
            public static final double encoderRatio = -0.5;  // Negate multiplier since encoder was inverted before
            public static final boolean encoderInverted = false; // Negate multiplier since encoder was inverted before
            public static final double encoderOffset = 0.2405;

            // PID constants
            public static final double kP = 20;
            public static final double kI = 0;
            public static final double kD = 0.5;

            // Feedforward constants
            public static final double kS = Robot.isSimulation() ? 0.0 : 0.12;
            public static final double kG = 0.36;
            public static final double kV = 0.88;
            public static final double kA = 0.02;

            public static final double maxVelocity = 0; // m/s
            public static final double maxAcceleration = 0; // m/s^2
            //public static final double maxJerk = 0; // m/s^3

            // Elevator tolerance
            public static final double elevatorTolerance = 0.05;

            // Simulation constants
            public static final double gearRatio = 8; // gear ratio
            public static final double carriageMass = 13.0; // in kg
            public static final double drumRadius = 0.022; // in meters
            public static final double minHeight = 0; // in meters
            public static final double maxHeight = 1.2; // in meters
            public static final DCMotor motorSim = DCMotor.getFalcon500Foc(2);

            // Motion Profile constants
            public static final Constraints pidConstraints= new Constraints(2.5,8);
            public static final double rotorToSensorRatio = gearRatio/(encoderRatio);//+encoderOffset

    }

    LoggedTalonFX leftMotor;
    LoggedTalonFX rightMotor;
    LoggedCANcoder encoder;

    LoggedMechanism2d mech = new LoggedMechanism2d(1.0,5.0);

    LoggedMechanismLigament2d root = new LoggedMechanismLigament2d("ElevatorRoot",0, 0);
    private final LoggedMechanismLigament2d elevator =
        root.append(new LoggedMechanismLigament2d("Elevator", 1.0, 0, 6, new Color8Bit(Color.kRed)));

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
    
    public Catvator(TalonFXIO motorIO, TalonFXIO motorIO2, CANcoderIO encoderIO){
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = 
            Constants.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        config.Slot0.kP = Constants.kP;
        config.Slot0.kI = Constants.kI;
        config.Slot0.kD = Constants.kD;
        config.Slot0.kG = Constants.kG;
        config.Slot0.kS = Constants.kS;
        config.Slot0.kV = Constants.kV;
        config.Slot0.kA = Constants.kA;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        
        
        config.Feedback.FeedbackRemoteSensorID = Constants.encoderId;
        config.Feedback.RotorToSensorRatio = Constants.rotorToSensorRatio;
        config.Feedback.SensorToMechanismRatio = Constants.encoderRatio;
        
        config.MotionMagic.MotionMagicCruiseVelocity = Constants.maxVelocity;
        config.MotionMagic.MotionMagicAcceleration = Constants.maxAcceleration;
        //config.MotionMagic.MotionMagicJerk = Constants.maxJerk;

        //motorConfig.CurrentLimits.StatorCurrentLimit = 0;
        //motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        // Inverts the encoder depending on Constants.encoderInverted
        encoderConfig.MagnetSensor.SensorDirection = Constants.encoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

        // Create logged motors and encoders from the configs
        leftMotor = new LoggedTalonFX(motorIO, "Elevator/LeftMotor", config);
        rightMotor = new LoggedTalonFX(motorIO, "Elevator/RightMotor", config);
        encoder = new LoggedCANcoder(encoderIO, "Elevator/Encoder", encoderConfig);
    }
}
