package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;

public class EncoderIOCANcoder extends EncoderIO {
    private CANcoder encoder;
    private CANcoderConfiguration config = new CANcoderConfiguration();
    private boolean configChanged = true;

    private double encoderRatio = 1;

    private CANcoderSimState sim;

    public EncoderIOCANcoder(int id, CANBus canBus) {
        encoder = new CANcoder(id, canBus);
        sim = encoder.getSimState();
    }

    public EncoderIOCANcoder(int id, String canBus) {
        this(id, new CANBus(canBus));
    }

    public EncoderIOCANcoder(int id) {
        this(id, new CANBus());
    }

    @Override
    public void updateInputs() {
        if (configChanged) {
            configChanged = false;
            encoder.getConfigurator().apply(config);
        }

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
    }

    // Sets the ratio and offset of this encoder. The ratio is (encoder radians)/(mechanism unit), and the offset is
    // measured in mechanism units. Note: this is purely aesthetic (it modifies the displayed encoder reading in AdvntageScope) so changing it won't affect actual robot behavior.
    @Override
    public void setRatio(double ratio) {
        encoderRatio = ratio;
    }

    // Sets whether the encoder is inverted
    @Override
    public void setInverted(boolean inverted) {
        config.MagnetSensor.SensorDirection =
                inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        configChanged = true;
    }

    @Override
    public void setMechPosition(double position) {
        sim.setRawPosition(Units.radiansToRotations(position * encoderRatio));
    }

    @Override
    public void setMechVelocity(double velocity) {
        sim.setVelocity(Units.radiansToRotations(velocity * encoderRatio));
    }
}
