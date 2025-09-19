package frc.robot.network;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.Wrist;

// Class that publishes 3D robot data to AdvantageScope
public class RobotPublisher {
    Arm arm;
    Wrist wrist;
    Intake intake;
    Elevator elevator;
    Hang hang;
    Swerve swerve;

    public RobotPublisher(Arm arm, Wrist wrist, Intake intake, Elevator elevator, Hang hang, Swerve swerve) {
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
        this.elevator = elevator;
        this.hang = hang;
        this.swerve = swerve;
    }

    public void publish() {}
}
