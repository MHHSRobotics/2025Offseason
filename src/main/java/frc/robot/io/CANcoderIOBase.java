package frc.robot.io;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants;

public class CANcoderIOBase extends CANcoderIO {
    private Debouncer connectedDebounce = new Debouncer(Constants.debounceTime);

    private CANcoder encoder;

    private StatusSignal<Angle> position;
    private StatusSignal<AngularVelocity> velocity;

    public CANcoderIOBase(int encoderId, String canBus) {
        encoder = new CANcoder(encoderId, canBus);
        position = encoder.getPosition();
        velocity = encoder.getVelocity();
    }

    public CANcoderIOBase(int encoderId) {
        this(encoderId, "");
    }

    @Override
    public void updateInputs() {
        CANcoderIOInputs inputs = getInputs();
        updateSimulation();
        StatusCode sc = BaseStatusSignal.refreshAll(position, velocity);
        inputs.connected = connectedDebounce.calculate(sc.isOK());
        inputs.position = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocity = Units.rotationsToRadians(velocity.getValueAsDouble());
    }

    public void updateSimulation() {}

    @Override
    public void applyConfig(CANcoderConfiguration config) {
        encoder.getConfigurator().apply(config);
    }

    public CANcoder getCANcoder() {
        return encoder;
    }
}
