package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Catvator;

public class ElevatorCommands {
    private Catvator catvator;
    public ElevatorCommands(Catvator catvator) {
        this.catvator = catvator;
    }
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(()-> catvator.setSpeed(speed.getAsDouble()));
    }

    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> catvator.setGoal(goal.getAsDouble()));
    }

    public Command changeGoal(DoubleSupplier change) {
        return new InstantCommand(()->catvator.setGoal(catvator.getGoal() + change.getAsDouble()));
    }
    public Command stop() {
        return setSpeed(()->0);
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return catvator.getSysId().quasistatic(dir).until(() -> !catvator.withinSysIdLimits());
    }

    // Returns a command that runs dynamic SysId for the arm in the given direction
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return catvator.getSysId().dynamic(dir).until(() -> !catvator.withinSysIdLimits());
    }
    
}
