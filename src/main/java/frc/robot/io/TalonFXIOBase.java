package frc.robot.io;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import frc.robot.Constants;

public class TalonFXIOBase extends TalonFXIO {
    private TalonFX motor;
    private Debouncer connectedDebounce = new Debouncer(Constants.debounceTime);

    private StatusSignal<Angle> position;
    private StatusSignal<AngularVelocity> velocity;
    private StatusSignal<Voltage> appliedVoltage;
    private StatusSignal<Voltage> supplyVoltage;
    private StatusSignal<Current> supplyCurrent;
    private StatusSignal<Current> statorCurrent;
    private StatusSignal<Temperature> temp;
    private StatusSignal<Double> closedLoopReference;
    private StatusSignal<Double> closedLoopError;
    private StatusSignal<ControlModeValue> controlMode;

    private DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0);

    public TalonFXIOBase(int motorId, String canBus) {
        motor = new TalonFX(motorId, canBus);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        supplyVoltage = motor.getSupplyVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        temp = motor.getDeviceTemp();
        closedLoopReference = motor.getClosedLoopReference();
        controlMode = motor.getControlMode();
        closedLoopError = motor.getClosedLoopError();
    }

    public TalonFXIOBase(int motorId) {
        this(motorId, "");
    }

    @Override
    public void updateInputs(TalonFXIOInputs inputs) {
        updateSimulation();
        StatusCode sc = BaseStatusSignal.refreshAll(
                position,
                velocity,
                appliedVoltage,
                supplyVoltage,
                supplyCurrent,
                statorCurrent,
                temp,
                closedLoopReference,
                controlMode,
                closedLoopError);
        inputs.connected = connectedDebounce.calculate(sc.isOK());
        inputs.position = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocity = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyVoltage = supplyVoltage.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.temp = temp.getValueAsDouble();
        inputs.setpoint = Units.rotationsToRadians(closedLoopReference.getValueAsDouble());
        inputs.controlMode = controlMode.getValue().toString();
        inputs.error = Units.rotationsToRadians(closedLoopError.getValueAsDouble());
    }

    public void updateSimulation() {}

    @Override
    public void applyConfig(TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
    }

    @Override
    public void setSpeed(double value) {
        motor.setControl(dutyCycleRequest.withOutput(value));
    }

    @Override
    public void setGoal(double pos) {
        motor.setControl(positionRequest.withPosition(pos));
    }

    protected TalonFX getMotor() {
        return motor;
    }
}
