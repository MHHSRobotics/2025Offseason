package frc.robot.io;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import org.littletonrobotics.junction.AutoLog;

public class MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public boolean connected;

        // Mechanism units are radians for arms and flywheels and meters for elevators.
        public double position; // mechanism units
        public double velocity; // mechanism units per sec
        public double accel; // mechanism units per sec^2

        public double appliedVoltage; // volts
        public double supplyVoltage; // volts
        public double supplyCurrent; // Battery current draw, amps
        public double torqueCurrent; // Motor torque indicator, amps

        public String controlMode; // Current control type

        public double setpoint; // Current setpoint for the motor, mechanism units
        public double error; // Difference between current position and goal position, mechanism units
        public double feedforward; // Current feedforward for the motor, amps
        public double derivOutput; // Current motor output from kD, amps
        public double intOutput; // Current motor output from kI, amps
        public double propOutput; // Current motor output from kP, amps

        public double temp; // Temperature, celsius

        public double dutyCycle; // Current duty cycle for the motor, -1 to 1

        public boolean hardwareFault;
        public boolean tempFault;
        public boolean forwardLimitFault;
        public boolean reverseLimitFault;
    }

    protected MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    public void updateInputs() {}

    public MotorIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public void setSpeed(double value) {}

    public void setVoltage(double volts) {}

    public void setTorqueCurrent(double current) {}

    public void setGoalWithCurrent(double goal) {}

    public void setGoalWithVoltage(double goal) {}

    public void setVelocityWithCurrent(double velocity) {}

    public void setVelocityWithVoltage(double velocity) {}

    public void follow(int motorId, boolean invert) {}

    public void setInverted(boolean inverted) {}

    public void setBraking(boolean braking) {}

    public void setkP(double kP) {}

    public void setkD(double kD) {}

    public void setkI(double kI) {}

    public void setkG(double kG) {}

    public void setkS(double kS) {}

    public void setkV(double kV) {}

    public void setkA(double kA) {}

    public void setGains(Slot0Configs gainss) {}

    public void setMaxVelocity(double maxVelocity) {}

    public void setMaxAccel(double maxAccel) {}

    public void setContinuousWrap(boolean wrap) {}

    public void setFeedforwardType(GravityTypeValue type) {}

    public void connectCANcoder(int id, double motorToSensorRatio, double sensorToMechanismRatio) {}

    public void setGearRatio(double gearRatio) {}

    public void setStatorCurrentLimit(double statorCurrentLimit) {}

    public void setSupplyCurrentLimit(double supplyCurrentLimit) {}

    public void setSupplyCurrentLowerLimit(double supplyCurrentLowerLimit) {}

    public void setSupplyCurrentLowerTime(double supplyCurrentLowerTime) {}

    public void setForwardLimit(double forwardLimit) {}

    public void setReverseLimit(double reverseLimit) {}

    public void setMechPosition(double position) {}

    public void setMechVelocity(double velocity) {}
}
