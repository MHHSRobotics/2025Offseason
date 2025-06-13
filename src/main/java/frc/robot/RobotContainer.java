package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.ArmCommands;
import frc.robot.io.TalonFXIO;
import frc.robot.subsystems.Arm;

public class RobotContainer {
    private final Arm arm;
    private final ArmCommands armCommands;
    private final CommandPS5Controller controller=new CommandPS5Controller(0);

    public RobotContainer() {
        arm=new Arm(new TalonFXIO() {
            
        });
        armCommands=new ArmCommands(arm);
        configureBindings();
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
