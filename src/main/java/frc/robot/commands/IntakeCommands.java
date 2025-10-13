package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.intake.Intake;

// Class to get commands for the intake mechanism. Simple speed control for picking up and ejecting game pieces.
public class IntakeCommands {
    private Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    // Tell the intake motor how fast to spin (percent [-1 to 1], -1 = full outtake, 1 = full intake)
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> intake.setSpeed(speed.getAsDouble()), intake).withName("intake set speed");
    }

    // Tell the intake motor to stop all movement
    public Command stop() {
        return setSpeed(() -> 0.0).withName("intake stop");
    }

    // Tell the intake to run at full speed (for picking up game pieces)
    public Command intake() {
        return setSpeed(() -> -1.0).withName("intake");
    }

    // Tell the intake to run in reverse at full speed (for ejecting game pieces)
    public Command outtake() {
        return setSpeed(() -> 1.0).withName("outtake");
    }

    // Tell the intake to pulse briefly (for game piece adjustment)
    public Command pulseBrief() {
        return setSpeed(() -> -0.5).withTimeout(0.1).andThen(stop()).withName("intake pulse brief");
    }

    // Tell the intake to reverse pulse briefly (for game piece adjustment)
    public Command reversePulseBrief() {
        return setSpeed(() -> 0.5).withTimeout(0.1).andThen(stop()).withName("intake reverse pulse brief");
    }
}
