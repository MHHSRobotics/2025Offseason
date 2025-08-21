package frc.robot.subsystems; // test

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.signals.GravityTypeValue;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import frc.robot.Robot;
import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;

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
        public static final boolean rightInverted = false;

        public static final double offset = 0.0;
        public static final double rightOffset = 0.0;

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

        public static final double MotorToSensorRatio = 0.0;
        public static final double SensorToMechanismRatio = 0.0;

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

    MotorIO leftMotor;
    MotorIO rightMotor;
    EncoderIO encoder;

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
                            .angularPosition(Radians.of(leftMotor.getInputs().position))
                            .angularVelocity(RadiansPerSecond.of(leftMotor.getInputs().velocity))
                            .voltage(Volts.of(leftMotor.getInputs().appliedVoltage)),
                    this));

    public Catvator(MotorIO leftMotorIO, MotorIO rightMotorIO, EncoderIO encoderIO) {
        leftMotor = leftMotorIO;
        rightMotor = rightMotorIO;
        encoder = encoderIO;

        leftMotor.setInverted(Constants.inverted);
        leftMotor.connectCANcoder(
                Constants.leftMotorId, Constants.MotorToSensorRatio, Constants.SensorToMechanismRatio);
        leftMotor.setOffset(Constants.offset);

        leftMotor.setFeedforwardType(GravityTypeValue.Elevator_Static);

        rightMotor.setInverted(Constants.rightInverted);
        rightMotor.connectCANcoder(
                Constants.rightMotorId, Constants.MotorToSensorRatio, Constants.SensorToMechanismRatio);
        rightMotor.setOffset(Constants.rightOffset);

        rightMotor.setFeedforwardType(GravityTypeValue.Elevator_Static);

        encoder.setInverted(Constants.encoderInverted);
        encoder.setRatio(Constants.encoderRatio);
    }

    public void setSpeed(double speed) {
        leftMotor.setSpeed(speed);
    }

    public void setGoal(double pos) {
        leftMotor.setGoalWithVoltageMagic(MathUtil.clamp(pos, Constants.minHeight, Constants.maxHeight));
    }

    public double getGoal() {
        return leftMotor.getInputs().setpoint;
    }

    public SysIdRoutine getSysId() {
        return sysId;
    }

    public boolean withinSysIdLimits() {
        return leftMotor.getInputs().position < Constants.maxSysIdHeight
                && leftMotor.getInputs().position > Constants.minSysIdHeight;
    }

    @Override
    public void periodic() {
        // Call periodic methods
        leftMotor.updateInputs();
        encoder.updateInputs();

        // Set angles of the visualization arm and goal arm
        elevator.setAngle(Rotation2d.fromRadians(leftMotor.getInputs().position));

        if (leftMotor.getInputs().controlMode.startsWith("MotionMagic")) {
            // If motor is currently in PID mode, show all the lines
            goalElevator.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);

            // Set the angles/lengths of the lines
            goalElevator.setLength(getGoal());
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
