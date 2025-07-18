package frc.robot.subsystems.swerve;

import frc.robot.io.CANcoderIO;
import frc.robot.io.LoggedCANcoder;
import frc.robot.io.LoggedTalonFX;
import frc.robot.io.TalonFXIO;

public class SwerveModule {
    private LoggedTalonFX driveMotor;
    private LoggedTalonFX angleMotor;
    private LoggedCANcoder angleEncoder;

    public SwerveModule(TalonFXIO driveMotor, TalonFXIO angleMotor, CANcoderIO angleEncoder) {
        
    }
}
