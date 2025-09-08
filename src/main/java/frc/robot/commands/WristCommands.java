package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.wrist.Wrist;

// Class to get commands for the wrist. Separate from Wrist for organizational purposes.
public class WristCommands {
    private Wrist wrist;

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }

    // Tell the wrist motor how fast to spin (percent [-1 to 1], -1 = full backward, 1 = full forward)
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> wrist.setDutyCycle(speed.getAsDouble()), wrist);
    }

    // Tell the wrist to go to a target angle (radians, like 0 = straight forward)
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> wrist.setGoal(goal.getAsDouble()), wrist);
    }

    // Tell the wrist to change its angle by the given amount (radians, positive = up, negative = down)
    public Command changeGoal(DoubleSupplier change) {
        return new InstantCommand(() -> wrist.setGoal(wrist.getGoal() + change.getAsDouble()), wrist);
    }

    // Tell the wrist to stop all motor output
    public Command stop() {
        return setSpeed(() -> 0);
    }

    // Tell the wrist to go to the straight forward position (0 radians = 0°)
    public Command goToStraight() {
        return setGoal(() -> 0.0);
    }

    // Tell the wrist to go to the up position (90° = π/2 radians)
    public Command goToUp() {
        return setGoal(() -> Units.degreesToRadians(90));
    }

    // Tell the wrist to go to the down position (-90° = -π/2 radians)
    public Command goToDown() {
        return setGoal(() -> Units.degreesToRadians(-90));
    }

    // Tell the wrist to go to the stow position (-135° for safe storage)
    public Command goToStow() {
        return setGoal(() -> Units.degreesToRadians(-135));
    }

    // Tell the wrist to move up by a small amount (5°)
    public Command nudgeUp() {
        return changeGoal(() -> Units.degreesToRadians(5));
    }

    // Tell the wrist to move down by a small amount (-5°)
    public Command nudgeDown() {
        return changeGoal(() -> Units.degreesToRadians(-5));
    }
}
