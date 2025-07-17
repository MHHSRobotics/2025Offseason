package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
    private Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    public Command stop() {
        return new InstantCommand(() -> intake.stopMotor());
    }

    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> intake.setSpeed(speed.getAsDouble()));
    }

    public Command intake() {
        Command command = new SequentialCommandGroup(
            new InstantCommand(() -> intake.setSpeed(Intake.Constants.intakeSpeed)), // Start the intake
            new WaitCommand(Intake.Constants.intakeTime),                                                   // Wait for 1 second
            stop()                                                                // Stop the intake
        );
        command.addRequirements(intake);
        return command;
    }

     public Command pulseIntake() {
        Command command = new RepeatCommand(
            new SequentialCommandGroup(
                setSpeed(() -> Intake.Constants.intakeSpeed),
                new WaitCommand(0.1),
                stop(),
                new WaitCommand(0.15)
            )                                                          // Stop the intake
        );
        command.addRequirements(intake);
        return command;
    }

    public Command outtake() {
        Command command = new SequentialCommandGroup(
            new InstantCommand(() -> intake.setSpeed(-Intake.Constants.intakeSpeed)), // Start the intake
            new WaitCommand(Intake.Constants.outtakeTime),                                                   // Wait for 1 second
            stop()                                                                // Stop the intake
        );
        command.addRequirements(intake);
        return command;
    }

    public Command outtake(DoubleSupplier time) {
        Command command = new SequentialCommandGroup(
            new InstantCommand(() -> intake.setSpeed(-Intake.Constants.intakeSpeed)), // Start the intake
            new WaitCommand(time.getAsDouble()),                                                   // Wait for 1 second
            stop()                                                                // Stop the intake
        );
        command.addRequirements(intake);
        return command;
    }
}
