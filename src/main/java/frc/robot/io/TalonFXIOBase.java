package frc.robot.io;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

// TalonFXIO implementation that interfaces with a physical TalonFX. In sim this interfaces with a simulated TalonFX.
public class TalonFXIOBase extends TalonFXIO {
    // The actual TalonFX
    private TalonFX motor;

    // All data here is measured in mech units, which are radians for arms and meters for elevators. TalonFX's native
    // signals are scaled by 2pi, so they have to be fixed when the inputs are recorded.

    public TalonFXIOBase(int motorId, String canBus) {
        motor = new TalonFX(motorId, canBus);
    }

    public TalonFXIOBase(int motorId) {
        this(motorId, "");
    }

    // Updates the TalonFXIOInputs
    @Override
    public void updateInputs(TalonFXIOInputs inputs) {
        // Update all the inputs from the signal values
        inputs.connected = motor.isConnected();

        // Some signals give rotations, so they have to be converted to radians
        inputs.position = Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
        inputs.velocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.accel = Units.rotationsToRadians(motor.getAcceleration().getValueAsDouble());

        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
        inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
        inputs.torqueCurrent = motor.getTorqueCurrent().getValueAsDouble();

        inputs.controlMode = motor.getControlMode().getValue().toString();

        inputs.setpoint =
                Units.rotationsToRadians(motor.getClosedLoopReference().getValueAsDouble());
        inputs.error = Units.rotationsToRadians(motor.getClosedLoopError().getValueAsDouble());
        inputs.feedforward = motor.getClosedLoopFeedForward().getValueAsDouble();
        inputs.derivOutput = motor.getClosedLoopDerivativeOutput().getValueAsDouble();
        inputs.intOutput = motor.getClosedLoopIntegratedOutput().getValueAsDouble();
        inputs.propOutput = motor.getClosedLoopProportionalOutput().getValueAsDouble();

        inputs.temp = motor.getDeviceTemp().getValueAsDouble();
        inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();

        inputs.hardwareFault = motor.getFault_Hardware().getValue();
        inputs.procTempFault = motor.getFault_ProcTemp().getValue();
        inputs.deviceTempFault = motor.getFault_DeviceTemp().getValue();
        inputs.undervoltageFault = motor.getFault_Undervoltage().getValue();
        inputs.bootDuringEnable = motor.getFault_BootDuringEnable().getValue();
        inputs.forwardHardLimit = motor.getFault_ForwardHardLimit().getValue();
        inputs.forwardSoftLimit = motor.getFault_ForwardSoftLimit().getValue();
        inputs.reverseHardLimit = motor.getFault_ReverseHardLimit().getValue();
        inputs.reverseSoftLimit = motor.getFault_ReverseSoftLimit().getValue();
    }

    @Override
    public void applyConfig(TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
    }

    @Override
    public void setControl(DutyCycleOut control) {
        motor.setControl(control);
    }

    @Override
    public void setControl(VoltageOut control) {
        motor.setControl(control);
    }

    @Override
    public void setControl(TorqueCurrentFOC control) {
        motor.setControl(control);
    }

    @Override
    public void setControl(MotionMagicTorqueCurrentFOC control) {
        motor.setControl(control);
    }

    @Override
    public void setControl(MotionMagicVoltage control) {
        motor.setControl(control);
    }

    @Override
    public void setControl(MotionMagicVelocityVoltage control) {
        motor.setControl(control);
    }

    @Override
    public void setControl(MotionMagicVelocityTorqueCurrentFOC control) {
        motor.setControl(control);
    }

    @Override
    public void setControl(Follower control) {
        motor.setControl(control);
    }

    public TalonFX getMotor() {
        return motor;
    }
}
