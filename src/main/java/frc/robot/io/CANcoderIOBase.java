package frc.robot.io;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Radians;

// CANcoderIO implementation that interfaces with a physical CANcoder
public class CANcoderIOBase extends CANcoderIO {
    // Debounce to make sure motor disconnects are real
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

    // Offset of the encoder in mechanism radians
    private double offset;

    // Ratio of encoder rotations to mechanism rotations
    private double encoderRatio;

    public CANcoderIOBase(int encoderId, String canBus, double encoderRatio, double offset) {
        encoder = new CANcoder(encoderId, canBus);

        this.offset = offset;
        this.encoderRatio = encoderRatio;

        // Initialize status signals
        position = encoder.getPosition();
        velocity = encoder.getVelocity();

        badMagnetFault = encoder.getFault_BadMagnet();
        hardwareFault = encoder.getFault_Hardware();
        bootDuringEnable = encoder.getFault_BootDuringEnable();
        undervoltage = encoder.getFault_Undervoltage();
    }

    public CANcoderIOBase(int encoderId, double encoderRatio, double offset) {
        this(encoderId, "", encoderRatio, offset);
    }

    @Override
    public void updateInputs(CANcoderIOInputs inputs) {
        // Update the simulation if we're in one
        updateSimulation();

        // Refresh all the status signals
        BaseStatusSignal.refreshAll(position, velocity, badMagnetFault, hardwareFault, bootDuringEnable, undervoltage);

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

    // Updates the simulation. Does nothing here, but subclasses can override this.
    public void updateSimulation() {}

    @Override
    public void applyConfig(CANcoderConfiguration config) {
        // Add the offset to the CANcoder config. The offset is in mechanism radians, so it needs to be converted to
        // encoder radians, then rotations.
        config.MagnetSensor.withMagnetOffset(Radians.of(offset * encoderRatio));

        // Apply the config
        encoder.getConfigurator().apply(config);
    }

    public CANcoder getCANcoder() {
        return encoder;
    }
}
