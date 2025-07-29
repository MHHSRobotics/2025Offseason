package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.io.GyroIO;

public class Swerve extends SubsystemBase {
    public static class Constants {}

    private SwerveModule[] modules = new SwerveModule[4];

    public Swerve(GyroIO gyro, SwerveModule[] modules) {
        this.modules = modules;
    }
}
