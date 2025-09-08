package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.arm.Arm;

// Class to get commands for the arm. Separate from Arm for organizational purposes.
public class ArmCommands {
    private Arm arm;

    public ArmCommands(Arm arm) {
        this.arm = arm;
    }

    // Command to set the speed of the arm
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> arm.setDutyCycle(speed.getAsDouble()), arm);
    }

    // Command to set the goal of the arm (in radians)
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> arm.setGoal(goal.getAsDouble()), arm);
    }

    // Command to change the goal of the arm by the given amount of radians
    public Command changeGoal(DoubleSupplier change) {
        return new InstantCommand(() -> arm.setGoal(arm.getGoal() + change.getAsDouble()), arm);
    }

    // Command to stop all motor output to the arm
    public Command stop() {
        return setSpeed(() -> 0);
    }
}
