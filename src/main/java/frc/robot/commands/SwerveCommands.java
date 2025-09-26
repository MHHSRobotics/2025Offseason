package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.swerve.Swerve;

public class SwerveCommands {
    private Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    // Stops the movement of the swerve drive
    public Command stop() {
        return new InstantCommand(() -> swerve.stop(), swerve).withName("swerve stop");
    }

    // Puts the swerve drive into an X position so it can't be pushed
    public Command lock() {
        return new InstantCommand(() -> swerve.lock(), swerve).withName("swerve lock");
    }

    public Command resetGyro() {
        return new InstantCommand(() -> swerve.resetGyro());
    }

    // Drives using the given dx, dy, omega, and field relative inputs. Applies a deadband and scales the values.
    public Command drive(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier omega, BooleanSupplier fieldRelative) {
        return Commands.run(
                        () -> {
                            double x = dx.getAsDouble();
                            double y = dy.getAsDouble();

                            // Get the distance from (x,y) to the origin
                            double radius = Math.hypot(x, y);

                            // Apply the deadband to the radius and take it to the power of movePow for smoother control
                            double scale = Math.pow(
                                    MathUtil.applyDeadband(radius, Swerve.Constants.moveDeadband),
                                    Swerve.Constants.movePow);

                            // Gets the angle in radians of the line from (0,0) to (x,y)
                            double angle = Math.atan2(y, x);

                            // Get the scaled values of x and y
                            x = scale * Math.cos(angle);
                            y = scale * Math.sin(angle);
                            double rotation = omega.getAsDouble();

                            // Apply the deadband to the absolute value of rotation and take it to the power of turnPow
                            // for
                            // smoother control
                            double rotationScale = Math.pow(
                                    MathUtil.applyDeadband(Math.abs(rotation), Swerve.Constants.turnDeadband),
                                    Swerve.Constants.turnPow);

                            // Copy the sign of rotation to rotationScale to get the final rotation value
                            rotation = Math.copySign(rotationScale, rotation);

                            // Run the swerve drive with the given values of x, y, and rotation
                            swerve.runSpeeds(x, y, rotation, fieldRelative.getAsBoolean());
                        },
                        swerve)
                .withName("swerve drive");
    }
}
