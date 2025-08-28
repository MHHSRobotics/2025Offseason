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

// Make the arm subsystem move a single-jointed arm to targets and show helpful
// visuals for students tuning it. All angles are arm mechanism angles (radians).
public class Arm extends SubsystemBase {

    public static class Constants {
        // All angles in this class are arm mechanism angles (radians)

        // CAN device ID for the arm motor controller
        public static final int motorId = 22;
        // Angle offset (radians) to line up the absolute encoder zero with the real arm zero
        public static final double offset = -2.6;
        // Whether to flip motor direction (true means reverse forward/backward)
        public static final boolean motorInverted = false;

        // CAN device ID for the absolute encoder
        public static final int encoderId = 26;
        // Whether to flip encoder direction to match the arm positive direction
        public static final boolean encoderInverted = false;

        public static final double gearRatio = 700 / 9.; // Ratio of motor rotations to arm rotations (unitless)
        public static final double encoderRatio = 28 / 9.; // Ratio of encoder rotations to arm rotations (unitless)

        public static final LoggedTunableNumber kP =
                new LoggedTunableNumber("Arm/kP", 29.84); // (volts per radian) more voltage when farther from target
        public static final LoggedTunableNumber kD =
                new LoggedTunableNumber("Arm/kD", 3.9867); // (volts per rad/s) reacts to how fast error is changing

        public static final LoggedTunableNumber kS = new LoggedTunableNumber(
                "Arm/kS", 0.0138); // (volts) voltage to get arm moving (overcome static friction)
        public static final LoggedTunableNumber kG = new LoggedTunableNumber(
                "Arm/kG", 1.308); // (volts) voltage to hold the arm level (compensate gravity at 0 rad)
        public static final LoggedTunableNumber kV = new LoggedTunableNumber(
                "Arm/kV", 9.2006); // (volts per rad/s) voltage that scales with speed to overcome friction
        public static final LoggedTunableNumber kA = new LoggedTunableNumber(
                "Arm/kA", 0.76272); // (volts per rad/s^2) extra voltage to help with acceleration

        public static final LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
                "Arm/maxVelocity", 100); // (rad/s) Motion Magic max speed for moving to a target
        public static final LoggedTunableNumber maxAccel = new LoggedTunableNumber(
                "Arm/maxAccel", 200); // (rad/s^2) Motion Magic max acceleration for moving to a target

        public static final double statorCurrentLimit = 70; // (amps) limit on motor torque output
        public static final double supplyCurrentLimit = 60; // (amps) normal current limit pulled from battery
        public static final double supplyCurrentLowerLimit = 40; // (amps) reduce to this if over limit for some time
        public static final double supplyCurrentLowerTime = 0.3; // (seconds) time before lowering current limit

        public static final double moi = 4.8944; // (kg·m^2) how hard it is to rotate the arm
        public static final double mass = 10; // (kg) estimated arm mass for simulation

        public static final double minAngle = Units.degreesToRadians(-45); // (radians) soft lower limit (~-45°)
        public static final double maxAngle = Units.degreesToRadians(140); // (radians) soft upper limit (~140°)
        public static final double startAngle = Units.degreesToRadians(90); // (radians) start angle in sim (~90°)

        // Angle bounds for SysId tests (radians). If the arm hits a hard stop during SysId, the test fails.
        // Keep these slightly inside the real mechanical limits to leave a few degrees of safety.
        public static final double minSysIdAngle = Units.degreesToRadians(-30);
        public static final double maxSysIdAngle = Units.degreesToRadians(125);

        public static final double rotorToSensorRatio =
                gearRatio / encoderRatio; // Ratio of motor rotations to encoder rotations (unitless)
        public static final double armLength =
                Math.sqrt(3 * moi / mass); // Virtual arm length (meters) so WPILib sim behaves like our real arm

        public static final LoggedNetworkBoolean manualArm =
                new LoggedNetworkBoolean("ArmSettings/Manual Arm", false); // Toggle to enable manual control mode
    }

    // Arm motor interface; handles real robot and simulation for us
    private MotorIO motor;

    // Absolute encoder interface; also supports simulation automatically
    private EncoderIO encoder;

    // On-screen drawing of the arm for dashboards (length is visual only)
    private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);

    // The fixed base point for the arm drawing
    private final LoggedMechanismRoot2d root = mech.getRoot("ArmRoot", 1, 1.5);

    // The live arm drawing that rotates to match the arm angle (radians)
    private final LoggedMechanismLigament2d arm =
            root.append(new LoggedMechanismLigament2d("Arm", 1.0, 0, 6, new Color8Bit(Color.kRed)));

    // Drawing that shows the arm's target angle (radians)
    private final LoggedMechanismLigament2d goalArm =
            root.append(new LoggedMechanismLigament2d("GoalArm", 1.0, 0, 6, new Color8Bit(Color.kYellow)));

    // Base point for the proportional (P) bar visualization
    private final LoggedMechanismRoot2d pRoot = mech.getRoot("PRoot", 2.5, 2);

    // Base point for the derivative (D) bar visualization
    private final LoggedMechanismRoot2d dRoot = mech.getRoot("DRoot", 2.6, 2);

    // Base point for the feedforward (FF) bar visualization
    private final LoggedMechanismRoot2d fRoot = mech.getRoot("FRoot", 2.7, 2);

    // Proportional (P) amount bar
    private final LoggedMechanismLigament2d pAmount =
            pRoot.append(new LoggedMechanismLigament2d("PAmount", 1.0, 90, 6, new Color8Bit(Color.kBlue)));

    // Derivative (D) amount bar
    private final LoggedMechanismLigament2d dAmount =
            dRoot.append(new LoggedMechanismLigament2d("DAmount", 1.0, 90, 6, new Color8Bit(Color.kGreen)));

    // Feedforward (FF) amount bar
    private final LoggedMechanismLigament2d fAmount =
            fRoot.append(new LoggedMechanismLigament2d("FAmount", 1.0, 90, 6, new Color8Bit(Color.kWhite)));

    // SysId routine (test mode for measuring how the arm moves)
    private final SysIdRoutine sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(1).per(Second), // Ramp rate for quasistatic (volts per second)
                    Volts.of(4), // Step voltage for dynamic test (volts)
                    Seconds.of(10)), // Timeout (seconds)
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

        // Tell the motor which direction is forward (true = invert)
        motor.setInverted(Constants.motorInverted);
        // Tell the motor which encoder to use and how motor/encoder/arm relate (ratios are unitless)
        motor.connectCANcoder(Constants.encoderId, Constants.rotorToSensorRatio, Constants.encoderRatio);
        // Tell the motor the encoder zero offset (radians) so arm angles match real life
        motor.setOffset(Constants.offset);

        // Make the motor use cosine gravity compensation (more help when the arm is level)
        motor.setFeedforwardType(GravityTypeValue.Arm_Cosine);

        encoder = encoderIO;
        // Tell the encoder which direction is positive and the gear ratio to the arm
        encoder.setInverted(Constants.encoderInverted);
        encoder.setRatio(Constants.encoderRatio);
    }

    // Tell the arm motor how fast to spin (percent [-1 to 1], -1 = full backward, 1 = full forward)
    public void setSpeed(double value) {
        motor.setSpeed(value);
    }

    // Tell the arm to go to a target angle (radians). Example: 0 rad ≈ arm straight forward.
    // We clamp to safe limits so the arm won't try to drive past its allowed range.
    public void setGoal(double pos) {
        motor.setGoalWithVoltageMagic(MathUtil.clamp(pos, Constants.minAngle, Constants.maxAngle));
    }

    // Find out the current target angle (radians)
    public double getGoal() {
        return motor.getInputs().setpoint;
    }

    // Find out the SysId routine used for measuring how the arm moves (test mode)
    public SysIdRoutine getSysId() {
        return sysId;
    }

    // Find out if the arm is inside the safe angle range for SysId (radians)
    public boolean withinSysIdLimits() {
        return motor.getInputs().position < Constants.maxSysIdAngle
                && motor.getInputs().position > Constants.minSysIdAngle;
    }

    @Override
    public void periodic() {
        // This runs every robot loop (about 50 times per second) to update sensors,
        // show visuals, apply tuning numbers, and check for problems
        // 1) Update sensor/motor inputs so the latest values are available
        motor.updateInputs();
        encoder.updateInputs();

        Logger.processInputs("Arm/Motor", motor.getInputs());
        Logger.processInputs("Arm/Encoder", encoder.getInputs());

        // 2) Update the on-screen arm drawing to match the current arm angle (radians)
        arm.setAngle(Rotation2d.fromRadians(motor.getInputs().position));

        if (motor.getInputs().controlMode.startsWith("MotionMagic")) {
            // If the motor is using Motion Magic (PID to a target), show the target and P/D/FF bars
            goalArm.setLineWeight(6);
            pAmount.setLineWeight(6);
            dAmount.setLineWeight(6);
            fAmount.setLineWeight(6);

            // Set the target angle and how big each control term is (scaled down for drawing)
            goalArm.setAngle(Rotation2d.fromRadians(motor.getInputs().setpoint));
            pAmount.setLength(motor.getInputs().propOutput / 100);
            dAmount.setLength(motor.getInputs().derivOutput / 100);
            fAmount.setLength(motor.getInputs().feedforward / 100);
        } else {
            // Hide the target and P/D/FF bars when not using Motion Magic
            goalArm.setLineWeight(0);
            pAmount.setLineWeight(0);
            dAmount.setLineWeight(0);
            fAmount.setLineWeight(0);
        }

        // 3) Send the mechanism drawing to the logs/dashboard
        Logger.recordOutput("Arm/Mech", mech);

        // 4) Read tuning numbers and apply them to the motor controller (units noted above)
        motor.setkP(Constants.kP.get());
        motor.setkD(Constants.kD.get());
        motor.setkG(Constants.kG.get());
        motor.setkS(Constants.kS.get());
        motor.setkV(Constants.kV.get());
        motor.setkA(Constants.kA.get());
        motor.setMaxVelocity(Constants.maxVelocity.get());
        motor.setMaxAccel(Constants.maxAccel.get());

        // 5) Update alerts so they appear on the dashboard
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
