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
        return new InstantCommand(() -> intake.setSpeed(speed.getAsDouble()), intake);
    }

    // Tell the intake motor to stop all movement
    public Command stop() {
        return new InstantCommand(() -> intake.stop(), intake);
    }

    // Tell the intake to run at full speed (for picking up game pieces)
    public Command intakeFull() {
        return setSpeed(() -> 1.0);
    }

    // Tell the intake to run at half speed (for gentle pickup or feeding)
    public Command intakeSlow() {
        return setSpeed(() -> 0.5);
    }

    // Tell the intake to run in reverse at full speed (for ejecting game pieces)
    public Command outtakeFull() {
        return setSpeed(() -> -1.0);
    }

    // Tell the intake to run in reverse at half speed (for gentle ejection)
    public Command outtakeSlow() {
        return setSpeed(() -> -0.5);
    }

    // Tell the intake to run in reverse at quarter speed (for very gentle ejection or feeding)
    public Command outtakeGentle() {
        return setSpeed(() -> -0.25);
    }

    // Tell the intake to run forward at quarter speed (for very gentle intake)
    public Command intakeGentle() {
        return setSpeed(() -> 0.25);
    }

    // Tell the intake to pulse briefly (for game piece adjustment)
    public Command pulseBrief() {
        return intakeSlow().withTimeout(0.1).andThen(stop());
    }

    // Tell the intake to reverse pulse briefly (for game piece adjustment)
    public Command reversePulseBrief() {
        return outtakeSlow().withTimeout(0.1).andThen(stop());
    }
}
