package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;

public class Hang extends SubsystemBase {
    public static class Constants {
        public static final int encoderId = 3;
        public static final double hangSpeed=1.0;
        public static final int motorId = 25;
        public static final boolean motorInverted = true;
    }

    private LoggedTalonFX motor;

    public Hang(TalonFXIO motorIO, CANcoderIO encoderIO) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motor = new LoggedTalonFX(motorIO, "Hang/Motor", motorConfig);
        motor.setInverted(Constants.motorInverted);
    }

    public void setSpeed(double value) {
        motor.setSpeed(value);
    }

    public void stop() {
        motor.setSpeed(0);
    }

    public void periodic() {
        motor.periodic();
    }
}
