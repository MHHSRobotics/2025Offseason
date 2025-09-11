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

    private int id;
    private CANBus canBus;

    public EncoderIOCANcoder(int id, CANBus canBus) {
        encoder = new CANcoder(id, canBus);
        sim = encoder.getSimState();
        this.id = id;
        this.canBus = canBus;
    }

    public EncoderIOCANcoder(int id, String canBus) {
        this(id, new CANBus(canBus));
    }

    public EncoderIOCANcoder(int id) {
        this(id, new CANBus());
    }

    public int getId() {
        return id;
    }

    public CANBus getCANBus() {
        return canBus;
    }

    @Override
    public void update() {
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

        // Update alerts using the base class method (this checks all fault conditions and updates dashboard alerts)
        super.update();
    }

    // Sets the ratio and offset of this encoder. The ratio is (encoder radians)/(mechanism unit). Offset is in
    // mechanism radians.
    @Override
    public void setRatioAndOffset(double ratio, double offset) {
        encoderRatio = ratio;
        config.MagnetSensor.MagnetOffset = Units.radiansToRotations(offset) * ratio;
        configChanged = true;
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
        double encoderPos = Units.radiansToRotations(position * encoderRatio);
        encoderPos = config.MagnetSensor.SensorDirection.equals(SensorDirectionValue.Clockwise_Positive)
                ? -encoderPos
                : encoderPos;
        sim.setRawPosition(encoderPos);
    }

    @Override
    public void setMechVelocity(double velocity) {
        double encoderVel = Units.radiansToRotations(velocity * encoderRatio);
        encoderVel = config.MagnetSensor.SensorDirection.equals(SensorDirectionValue.Clockwise_Positive)
                ? -encoderVel
                : encoderVel;
        sim.setVelocity(encoderVel);
    }
}
