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
        return new InstantCommand(() -> wrist.setSpeed(speed.getAsDouble()), wrist).withName("wrist set duty cycle");
    }

    // Tell the wrist to go to a target angle (radians, like 0 = straight forward)
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> wrist.setGoal(goal.getAsDouble()), wrist).withName("wrist set goal");
    }

    // Tell the wrist to change its angle by the given amount (radians, positive = up, negative = down)
    public Command changeGoal(DoubleSupplier change) {
        return new InstantCommand(() -> wrist.setGoal(wrist.getGoal() + change.getAsDouble()), wrist)
                .withName("wrist change goal");
    }

    // Tell the wrist to stop all motor output
    public Command stop() {
        return setSpeed(() -> 0).withName("wrist stop");
    }

    // Tell the wrist to go to the straight forward position (0 radians = 0°)
    public Command goToStraight() {
        return setGoal(() -> Units.degreesToRadians(-25)).withName("wrist go to straight");
    }

    // Tell the wrist to go to the up position (90° = π/2 radians)
    public Command goToUp() {
        return setGoal(() -> Units.degreesToRadians(90)).withName("wrist go to up");
    }

    // Tell the wrist to go to the down position (-90° = -π/2 radians)
    public Command goToDown() {
        return setGoal(() -> Units.degreesToRadians(-90)).withName("wrist go to down");
    }

    // Command to manually control the wrist at a fixed speed
    public Command manualControl(double speed) {
        return setSpeed(() -> speed).withName("wrist manual control");
    }

    // Command to set the goal of the wrist to a fixed value (radians)
    public Command setGoal(double goal) {
        return setGoal(() -> goal).withName("wrist set goal " + goal);
    }

    // Command to increment the goal by a fixed amount (radians)
    public Command incrementGoal(double increment) {
        return changeGoal(() -> increment).withName("wrist increment goal");
    }
}
