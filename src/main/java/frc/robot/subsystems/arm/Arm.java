package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

import frc.robot.io.EncoderIO;
import frc.robot.io.MotorIO;
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

    // Motor, uses MotorIO to automatically handle simulation
    private MotorIO motor;

    // EncoderIO automatically handles simulation
    private EncoderIO encoder;

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
                            .angularPosition(Rotations.of(motor.getInputs().position))
                            .angularVelocity(RotationsPerSecond.of(motor.getInputs().velocity))
                            .voltage(Volts.of(motor.getInputs().appliedVoltage)),
                    this));

    private Alert motorDisconnect = new Alert("The arm motor is disconnected", AlertType.kError);
    private Alert motorHardwareFault =
            new Alert("The arm motor encountered an internal hardware fault", AlertType.kError);
    private Alert motorOverheat = new Alert("The arm motor is overheating!", AlertType.kWarning);
    private Alert motorForwardLimit = new Alert("The arm motor hit its forward limit", AlertType.kWarning);
    private Alert motorReverseLimit = new Alert("The arm motor hit its reverse limit", AlertType.kWarning);

    private Alert encoderDisconnect = new Alert("The arm encoder is disconnected", AlertType.kError);
    private Alert encoderHardwareFault =
            new Alert("The arm encoder encountered an internal hardware fault", AlertType.kError);
    private Alert encoderMagnetFault = new Alert("The arm encoder magnet is not functioning", AlertType.kError);

    public Arm(MotorIO motorIO, EncoderIO encoderIO) {
        motor = motorIO;

        motor.setInverted(Constants.motorInverted);
        motor.connectCANcoder(Constants.encoderId, Constants.rotorToSensorRatio, Constants.encoderRatio);

        motor.setFeedforwardType(GravityTypeValue.Arm_Cosine);

        encoder = encoderIO;
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
        return motor.getInputs().setpoint;
    }

    // Gets the SysId routine
    public SysIdRoutine getSysId() {
        return sysId;
    }

    // Checks if the arm is in safe SysId range
    public boolean withinSysIdLimits() {
        return motor.getInputs().position < Constants.maxSysIdAngle
                && motor.getInputs().position > Constants.minSysIdAngle;
    }

    @Override
    public void periodic() {
        // Call periodic methods
        motor.updateInputs();
        encoder.updateInputs();

        Logger.processInputs("Arm/Motor", motor.getInputs());
        Logger.processInputs("Arm/Encoder", encoder.getInputs());

        // Set angles of the visualization arm and goal arm
        arm.setAngle(Rotation2d.fromRadians(motor.getInputs().position));

        if (motor.getInputs().controlMode.startsWith("MotionMagic")) {
            // If motor is currently in PID mode, show all the lines
            goalArm.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);

            // Set the angles/lengths of the lines
            goalArm.setAngle(Rotation2d.fromRadians(motor.getInputs().setpoint));
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

        // Update the tuning constants
        motor.setkP(Constants.kP.get());
        motor.setkD(Constants.kD.get());
        motor.setkG(Constants.kG.get());
        motor.setkS(Constants.kS.get());
        motor.setkV(Constants.kV.get());
        motor.setkA(Constants.kA.get());
        motor.setMaxVelocity(Constants.maxVelocity.get());
        motor.setMaxAccel(Constants.maxAccel.get());

        // Update the alerts
        motorDisconnect.set(!motor.getInputs().connected);
        motorOverheat.set(motor.getInputs().tempFault);
        motorHardwareFault.set(motor.getInputs().hardwareFault);
        motorForwardLimit.set(motor.getInputs().forwardLimitFault);
        motorReverseLimit.set(motor.getInputs().reverseLimitFault);

        encoderDisconnect.set(!encoder.getInputs().connected);
        encoderHardwareFault.set(encoder.getInputs().hardwareFault);
        encoderMagnetFault.set(encoder.getInputs().badMagnetFault);
    }
}
