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

public class CANcoderIOBase extends CANcoderIO {
    private Debouncer connectedDebounce = new Debouncer(Constants.debounceTime);

    private CANcoder encoder;

    private StatusSignal<Angle> position;
    private StatusSignal<AngularVelocity> velocity;

    private StatusSignal<Boolean> badMagnetFault;
    private StatusSignal<Boolean> hardwareFault;
    private StatusSignal<Boolean> bootDuringEnable;
    private StatusSignal<Boolean> undervoltage;

    private double offset;
    private double encoderRatio;

    public CANcoderIOBase(int encoderId, String canBus, double encoderRatio, double offset) {
        encoder = new CANcoder(encoderId, canBus);

        this.offset = offset;
        this.encoderRatio = encoderRatio;

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
        updateSimulation();
        BaseStatusSignal.refreshAll(position, velocity, badMagnetFault, hardwareFault, bootDuringEnable, undervoltage);
        inputs.connected = connectedDebounce.calculate(encoder.isConnected());
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) / encoderRatio;
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / encoderRatio;
    }

    public void updateSimulation() {}

    @Override
    public void applyConfig(CANcoderConfiguration config) {
        config.MagnetSensor.withMagnetOffset(Radians.of(offset * encoderRatio));
        encoder.getConfigurator().apply(config);
    }

    public CANcoder getCANcoder() {
        return encoder;
    }
}
