package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.signals.GravityTypeValue;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedCANcoder;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase {

    public static class Constants {
        // All radians are mechanism radians

        public static final int motorId = 22;
        public static final boolean motorInverted = false;

        public static final int encoderId = 1;
        public static final double encoderOffset = -1.052; // radians
        public static final boolean encoderInverted = false;

        public static final double gearRatio = 700 / 9.; // ratio of motor rotations to mechanism rotations
        public static final double encoderRatio = 28 / 9.; // ratio of encoder rotations to mechanism rotations

        public static final LoggedTunableNumber kP =
                new LoggedTunableNumber("Arm/kP", 29.84); // volts per radian, current scales with distance to setpoint
        public static final LoggedTunableNumber kD =
                new LoggedTunableNumber("Arm/kD", 3.9867); // volts per radian per sec

        public static final LoggedTunableNumber kS =
                new LoggedTunableNumber("Arm/kS", 0.0138); // volts, the current needed to overcome static friction
        public static final LoggedTunableNumber kG = new LoggedTunableNumber(
                "Arm/kG", 1.308); // volts, the current needed to overcome gravity when the arm is horizontal
        public static final LoggedTunableNumber kV = new LoggedTunableNumber(
                "Arm/kV",
                9.2006); // volts per radian per sec, current needed to overcome linear friction (scales with velocity)
        public static final LoggedTunableNumber kA = new LoggedTunableNumber(
                "Arm/kA",
                0.76272); // volts per radian per sec^2, current needed to overcome quadratic friction (scales with
        // acceleration)

        public static final LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
                "Arm/maxVelocity", 100); // radians per sec, sets the max velocity MotionMagic will use
        public static final LoggedTunableNumber maxAccel = new LoggedTunableNumber(
                "Arm/maxAccel", 200); // radians per sec^2, sets the max acceleration MotionMagic will use

        public static final double statorCurrentLimit = 70; // Limit on total torque output from the motor
        public static final double supplyCurrentLimit = 60; // Limit on current pull from the motor
        public static final double supplyCurrentLowerLimit =
                40; // If the motor pulls >40 amps for >0.3 seconds, then the current limit will be set to 40 amps
        public static final double supplyCurrentLowerTime = 0.3;

        public static final double moi = 4.8944; // kg m^2, how hard it is to rotate the arm
        public static final double mass = 10; // kg

        public static final double minAngle = Units.degreesToRadians(-45);
        public static final double maxAngle = Units.degreesToRadians(140);
        public static final double startAngle = Units.degreesToRadians(90); // start angle for the simulated arm

        // Angle bounds for SysId tests. If the arm hits its physical limit during SysId then the identification will
        // fail. These bounds should be close to the mechanical bounds but with a few degrees of clearance.
        public static final double minSysIdAngle = Units.degreesToRadians(-30);
        public static final double maxSysIdAngle = Units.degreesToRadians(125);

        public static final double rotorToSensorRatio =
                gearRatio / encoderRatio; // ratio of motor rotations to encoder rotations
        public static final double armLength =
                Math.sqrt(3 * moi / mass); // math to make WPILib simulate the arm correctly

        public static final LoggedNetworkBoolean manualArm = new LoggedNetworkBoolean(
                "ArmSettings/Manual Arm", false); // toggles whether the arm is in manual control
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

    // SysId routine
    private final SysIdRoutine sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(1).per(Second), // Ramp rate for quasistatic, volts/sec
                    Volts.of(4), // Step voltage for dynamic
                    Seconds.of(10)), // Timeout
            new SysIdRoutine.Mechanism(
                    // Voltage setting function
                    (voltage) -> motor.setVoltage(voltage.in(Volts)),
                    // SysId logging function
                    (log) -> log.motor("arm")
                            .angularPosition(Rotations.of(motor.getPosition()))
                            .angularVelocity(RotationsPerSecond.of(motor.getVelocity()))
                            .voltage(Volts.of(motor.getInputs().appliedVoltage)),
                    this));

    public Arm(TalonFXIO motorIO, CANcoderIO encoderIO) {
        motor = new LoggedTalonFX(motorIO, "Arm/Motor");
        motor.setInverted(Constants.motorInverted);
        motor.connectCANcoder(Constants.encoderId, Constants.rotorToSensorRatio, Constants.encoderRatio);

        motor.setFeedforwardType(GravityTypeValue.Arm_Cosine);

        encoder = new LoggedCANcoder(encoderIO, "Arm/Encoder");
        encoder.setInverted(Constants.encoderInverted);
        encoder.setRatioAndOffset(Constants.encoderRatio, Constants.encoderOffset);
    }

    // Sets the speed of the arm. Speed is from -1 (full backward) to 1 (full forward)
    public void setSpeed(double value) {
        motor.setSpeed(value);
    }

    // Sets the goal of the arm in radians
    public void setGoal(double pos) {
        motor.setGoalWithVoltage(MathUtil.clamp(pos, Constants.minAngle, Constants.maxAngle));
    }

    // Returns the goal of the arm
    public double getGoal() {
        return motor.getGoal();
    }

    // Gets the SysId routine
    public SysIdRoutine getSysId() {
        return sysId;
    }

    // Checks if the arm is in safe SysId range
    public boolean withinSysIdLimits() {
        return motor.getPosition() < Constants.maxSysIdAngle && motor.getPosition() > Constants.minSysIdAngle;
    }

    @Override
    public void periodic() {
        // Call periodic methods
        motor.periodic();
        encoder.periodic();

        // Set angles of the visualization arm and goal arm
        arm.setAngle(Rotation2d.fromRadians(motor.getPosition()));

        if (motor.getInputs().controlMode.startsWith("MotionMagic")) {
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
            // Make all the lines invisible by setting their width to 0
            goalArm.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
        }

        // Log the mechanism
        Logger.recordOutput("Arm/Mech", mech);

        // Update the tuning constants if applicable
        if (Constants.kP.hasChanged(hashCode())) {
            motor.setkP(Constants.kP.get());
        }
        if (Constants.kD.hasChanged(hashCode())) {
            motor.setkD(Constants.kD.get());
        }
        if (Constants.kG.hasChanged(hashCode())) {
            motor.setkG(Constants.kG.get());
        }
        if (Constants.kS.hasChanged(hashCode())) {
            motor.setkS(Constants.kS.get());
        }
        if (Constants.kV.hasChanged(hashCode())) {
            motor.setkV(Constants.kV.get());
        }
        if (Constants.kA.hasChanged(hashCode())) {
            motor.setkA(Constants.kA.get());
        }
        if (Constants.maxVelocity.hasChanged(hashCode())) {
            motor.setMaxVelocity(Constants.maxVelocity.get());
        }
        if (Constants.maxAccel.hasChanged(hashCode())) {
            motor.setMaxAccel(Constants.maxAccel.get());
        }
    }
}
