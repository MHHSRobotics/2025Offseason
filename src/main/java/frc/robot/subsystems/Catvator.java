package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import frc.robot.Robot;
import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedCANcoder;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Catvator extends SubsystemBase {
    public static class Constants {
        // SysID var
        public static final int leftMotorId = 20;
        public static final int rightMotorId = 21;

        public static final boolean inverted = false;

        // Encoder ID
        public static final int encoderId = 2;
        public static final double encoderRatio = -0.5; // Negate multiplier since encoder was inverted before
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

        public static final int SupplyCurrentLimit = 40;
        public static final int MaxVolt = 12;

        public static final double maxVelocity = 0; // m/s
        public static final double maxAcceleration = 0; // m/s^2
        // public static final double maxJerk = 0; // m/s^3

        // Elevator tolerance
        public static final double elevatorTolerance = 0.05;

        // Simulation constants
        public static final double gearRatio = 8; // gear ratio
        public static final double carriageMass = 13.0; // in kg
        public static final double drumRadius = 0.022; // in meters

        public static final double minHeight = 0.1; // in meters
        public static final double maxHeight = 1.2; // in meters
        public static final double minSysIdHeight = 0.1;
        public static final double maxSysIdHeight = 1.0;

        public static final DCMotor motorSim = DCMotor.getFalcon500Foc(2);

        // Motion Profile constants
        public static final Constraints pidConstraints = new Constraints(2.5, 8);
        public static final double rotorToSensorRatio = gearRatio / (encoderRatio); // +encoderOffset
    }

    LoggedTalonFX leftMotor;
    LoggedTalonFX rightMotor;
    LoggedCANcoder encoder;

    LoggedMechanism2d mech = new LoggedMechanism2d(1.0, 5.0);

    LoggedMechanismLigament2d root = new LoggedMechanismLigament2d("ElevatorRoot", 0, 0);
    private final LoggedMechanismLigament2d elevator =
            root.append(new LoggedMechanismLigament2d("Elevator", 6, 0, 6, new Color8Bit(Color.kRed)));

    private final LoggedMechanismLigament2d goalElevator =
            root.append(new LoggedMechanismLigament2d("Elevator", 6, 0, 6, new Color8Bit(Color.kRed)));

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

    // SysId routine
    private final SysIdRoutine sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(1).per(Second), // Ramp rate for quasistatic, volts/sec
                    Volts.of(4), // Step voltage for dynamic
                    Seconds.of(10)), // Timeout
            new SysIdRoutine.Mechanism(
                    // Voltage setting function
                    (voltage) -> leftMotor.setVoltage(voltage.in(Volts)),
                    // SysId logging function
                    (log) -> log.motor("arm")
                            .angularPosition(Radians.of(leftMotor.getPosition()))
                            .angularVelocity(RadiansPerSecond.of(leftMotor.getVelocity()))
                            .voltage(Volts.of(leftMotor.getInputs().appliedVoltage)),
                    this));

    public Catvator(TalonFXIO motorIO, TalonFXIO motorIO2, CANcoderIO encoderIO) {

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

        config.ClosedLoopGeneral. = 0;

        config.ClosedLoopRamps. = 0;

        config.CurrentLimits.StatorCurrentLimit = Constants.SupplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CustomParams. = 0;

        config.DifferentialConstants.Peak = 0;

        config.DifferentialSensors.DifferentialSensorSource = DifferentialSensorSourceValue.RemoteCANcoder;
        config.DifferentialSensors.DifferentialRemoteSensorID = Constants.encoderId;

        config.Feedback.FeedbackRotorOffset = 0; //min -1 max 1
        config.Feedback.SensorToMechanismRatio = 1.0; //min -1000 max 1000
        config.Feedback.RotorToSensorRatio = Constants.rotorToSensorRatio; //min -1000 max 1000
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.FutureProofConfigs = true;

        config.HardwareLimitSwitch. = 0;

        config.MotionMagic. = 0;

        //config.MotorOutput. = 0;

        config.OpenLoopRamps. = 0;

        config.SoftwareLimitSwitch. = 0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 400;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -400; //min -800 max 800
        config.TorqueCurrent.TorqueNeutralDeadband = 0; //min 0 max 25

        config.Voltage.SupplyVoltageTimeConstant = 0;
        config.Voltage.PeakForwardVoltage = Constants.MaxVolt;
        config.Voltage.PeakReverseVoltage = -1 * Constants.MaxVolt;

        config.Feedback.FeedbackRemoteSensorID = Constants.encoderId;
        config.Feedback.RotorToSensorRatio = Constants.rotorToSensorRatio;
        config.Feedback.SensorToMechanismRatio = Constants.encoderRatio;

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.maxVelocity;
        config.MotionMagic.MotionMagicAcceleration = Constants.maxAcceleration;
        // config.MotionMagic.MotionMagicJerk = Constants.maxJerk;

        // motorConfig.CurrentLimits.StatorCurrentLimit = 0;
        // motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.SupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = Constants.MaxVolt;
        config.Voltage.PeakReverseVoltage = -1 * Constants.MaxVolt;

        config.deserialize(getName());
        config.serialize();
        config.toString();
        config.withClosedLoopGeneral(null);
        config.withClosedLoopRamps(null);
        config.withCurrentLimits(null);
        config.withCustomParams(null);
        config.withDifferentialConstants(null);
        config.withDifferentialSensors(null);
        config.withFeedback(null);
        config.withHardwareLimitSwitch(null);
        config.withMotionMagic(null);
        config.withMotorOutput(null);
        config.withOpenLoopRamps(null);
        config.withSlot0(null);
        config.withSoftwareLimitSwitch(null);
        config.withTorqueCurrent(null);
        config.withVoltage(null);
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        // Inverts the encoder depending on Constants.encoderInverted
        encoderConfig.MagnetSensor.SensorDirection = Constants.encoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

        // Create logged motors and encoders from the configs
        leftMotor = new LoggedTalonFX(motorIO, "Elevator/LeftMotor", config);
        rightMotor = new LoggedTalonFX(motorIO2, "Elevator/RightMotor", config);
        rightMotor.follow(Constants.leftMotorId, false);
        encoder = new LoggedCANcoder(encoderIO, "Elevator/Encoder", encoderConfig);
    }

    public void setSpeed(double speed) {
        leftMotor.setSpeed(speed);
    }

    public void setGoal(double pos) {
        leftMotor.setGoal(MathUtil.clamp(pos, Constants.minHeight, Constants.maxHeight));
    }

    public double getGoal() {
        return leftMotor.getGoal();
    }

    public SysIdRoutine getSysId() {
        return sysId;
    }

    public boolean withinSysIdLimits() {
        return leftMotor.getPosition() < Constants.maxSysIdHeight && leftMotor.getPosition() > Constants.minSysIdHeight;
    }

    @Override
    public void periodic() {
        // Call periodic methods
        leftMotor.periodic();
        encoder.periodic();

        // Set angles of the visualization arm and goal arm
        elevator.setAngle(Rotation2d.fromRadians(leftMotor.getPosition()));

        if (leftMotor.getInputs().controlMode.startsWith("MotionMagic")) {
            // If motor is currently in PID mode, show all the lines
            goalElevator.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);

            // Set the angles/lengths of the lines
            goalElevator.setLength(leftMotor.getGoal());
            pAmount.setLength(leftMotor.getInputs().propOutput / 100);
            dAmount.setLength(leftMotor.getInputs().derivOutput / 100);
            fAmount.setLength(leftMotor.getInputs().feedforward / 100);
        } else {
            // Make all the lines invisible by setting their width to 0
            goalElevator.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
        }

        // Log the mechanism
        Logger.recordOutput("Elevator/Mech", mech);
    }
}
