package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

// CANcoderIO implementation that interfaces with a physical CANcoder
public class CANcoderIOBase extends CANcoderIO {
    // The actual CANcoder
    private CANcoder encoder;

    // Ratio of encoder rotations to mechanism rotations
    private double encoderRatio = 1;

    public CANcoderIOBase(int encoderId, String canBus) {
        encoder = new CANcoder(encoderId, canBus);
    }

    public CANcoderIOBase(int encoderId) {
        this(encoderId, "");
    }

    @Override
    public void updateInputs(CANcoderIOInputs inputs) {
        inputs.connected = encoder.isConnected();

        // CANcoder signals give data in encoder rotations. First convert to radians, then use encoderRatio to convert
        // to mechanism units.
        inputs.positionRad =
                Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble()) / encoderRatio;
        inputs.velocityRadPerSec =
                Units.rotationsToRadians(encoder.getVelocity().getValueAsDouble()) / encoderRatio;

        // Update fault inputs
        inputs.badMagnetFault = encoder.getFault_BadMagnet().getValue();
        inputs.hardwareFault = encoder.getFault_Hardware().getValue();
        inputs.bootDuringEnable = encoder.getFault_BootDuringEnable().getValue();
        inputs.undervoltage = encoder.getFault_Undervoltage().getValue();
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
