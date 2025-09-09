package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.hang.Hang;

// Class to get commands for the hang mechanism. Simple speed control for climbing.
public class HangCommands {
    private Hang hang;

    public HangCommands(Hang hang) {
        this.hang = hang;
    }

    // Tell the hang motor how fast to spin (percent [-1 to 1], -1 = full down, 1 = full up)
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> hang.setSpeed(speed.getAsDouble()), hang);
    }

    // Tell the hang motor to stop all movement
    public Command stop() {
        return new InstantCommand(() -> hang.stop(), hang);
    }

    // Tell the hang to extend up at full speed (for climbing up)
    public Command extendUp() {
        return setSpeed(() -> 1.0);
    }

    // Tell the hang to retract down at full speed (for lowering down)
    public Command retractDown() {
        return setSpeed(() -> -1.0);
    }
}
