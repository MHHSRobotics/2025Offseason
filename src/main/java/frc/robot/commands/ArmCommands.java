package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.arm.Arm;

public class ArmCommands {
    private Arm arm;

    public ArmCommands(Arm arm) {
        this.arm = arm;
    }

    // Command to set the speed of the arm
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> arm.setSpeed(speed.getAsDouble()));
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

    // Returns a command that runs quasistatic SysId for the arm in the given direction
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return arm.getSysId().quasistatic(dir).until(() -> !arm.withinSysIdLimits());
    }

    // Returns a command that runs dynamic SysId for the arm in the given direction
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return arm.getSysId().dynamic(dir).until(() -> !arm.withinSysIdLimits());
    }
}
