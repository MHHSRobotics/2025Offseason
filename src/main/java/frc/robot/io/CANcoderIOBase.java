package frc.robot.io;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

// CANcoderIO implementation that interfaces with a physical CANcoder
public class CANcoderIOBase extends CANcoderIO {
    // Debounce to make sure encoder disconnects are real
    private Debouncer connectedDebounce = new Debouncer(Constants.debounceTime);

    // The actual CANcoder
    private CANcoder encoder;

    // Status signals to get data from the encoder
    private StatusSignal<Angle> position;
    private StatusSignal<AngularVelocity> velocity;

    private StatusSignal<Boolean> badMagnetFault;
    private StatusSignal<Boolean> hardwareFault;
    private StatusSignal<Boolean> bootDuringEnable;
    private StatusSignal<Boolean> undervoltage;

    // Ratio of encoder rotations to mechanism rotations
    private double encoderRatio = 1;

    public CANcoderIOBase(int encoderId, String canBus) {
        encoder = new CANcoder(encoderId, canBus);

        // Initialize status signals
        position = encoder.getPosition();
        velocity = encoder.getVelocity();

        badMagnetFault = encoder.getFault_BadMagnet();
        hardwareFault = encoder.getFault_Hardware();
        bootDuringEnable = encoder.getFault_BootDuringEnable();
        undervoltage = encoder.getFault_Undervoltage();

        // Register signals to be automatically refreshed every 20ms
        PhoenixUtil.registerSignals(
                canBus.equals("canivore"),
                position,
                velocity,
                badMagnetFault,
                hardwareFault,
                bootDuringEnable,
                undervoltage);
    }

    public CANcoderIOBase(int encoderId) {
        this(encoderId, "");
    }

    @Override
    public void updateInputs(CANcoderIOInputs inputs) {
        // Update the inputs
        inputs.connected = connectedDebounce.calculate(encoder.isConnected());

        // CANcoder signals give data in encoder rotations. First convert to radians, then use encoderRatio to convert
        // to mechanism units.
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) / encoderRatio;
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / encoderRatio;

        // Update fault inputs
        inputs.badMagnetFault = badMagnetFault.getValue();
        inputs.hardwareFault = hardwareFault.getValue();
        inputs.bootDuringEnable = bootDuringEnable.getValue();
        inputs.undervoltage = undervoltage.getValue();
    }

    @Override
    public void applyConfig(CANcoderConfiguration config) {
        encoder.getConfigurator().apply(config);
    }

    public CANcoder getCANcoder() {
        return encoder;
    }

    @Override
    public void setEncoderRatio(double encoderRatio) {
        this.encoderRatio = encoderRatio;
    }
}
