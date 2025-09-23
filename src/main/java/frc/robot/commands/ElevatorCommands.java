package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.elevator.Elevator;

// Class to get commands for the elevator. Separate from Elevator for organizational purposes.
public class ElevatorCommands {
    private Elevator elevator;

    public ElevatorCommands(Elevator elevator) {
        this.elevator = elevator;
    }

    // Tell the elevator motors how fast to spin (percent [-1 to 1], -1 = full down, 1 = full up)
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> elevator.setSpeed(speed.getAsDouble()), elevator)
                .withName("elevator set speed");
    }

    // Tell the elevator to go to a target height (meters, like 0.5 = half meter up)
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> elevator.setGoal(goal.getAsDouble()), elevator).withName("elevator set goal");
    }

    // Tell the elevator to change its height by the given amount (meters, positive = up, negative = down)
    public Command changeGoal(DoubleSupplier change) {
        return new InstantCommand(() -> elevator.setGoal(elevator.getGoal() + change.getAsDouble()), elevator)
                .withName("elevator change goal");
    }

    // Tell the elevator to stop all motor output
    public Command stop() {
        return setSpeed(() -> 0).withName("elevator stop");
    }

    // Tell the elevator to go to the bottom position (0 meters)
    public Command goToBottom() {
        return setGoal(() -> 0.0).withName("elevator go to bottom");
    }

    // Tell the elevator to go to the top position (1.2 meters)
    public Command goToTop() {
        return setGoal(() -> 1.2).withName("elevator go to top");
    }

    // Tell the elevator to go to the middle position (0.6 meters)
    public Command goToMiddle() {
        return setGoal(() -> 0.6).withName("elevator go to middle");
    }
}
