package frc.robot.network;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.Wrist;

// Class that publishes 3D robot data to AdvantageScope
public class RobotPublisher {
    private Arm arm;
    private Wrist wrist;
    private Intake intake;
    private Elevator elevator;
    private Hang hang;
    private Swerve swerve;

    public RobotPublisher(Arm arm, Wrist wrist, Intake intake, Elevator elevator, Hang hang, Swerve swerve) {
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
        this.elevator = elevator;
        this.hang = hang;
        this.swerve = swerve;
    }

    public void publish() {
        Pose2d pos = swerve.getPose();
        Pose3d pos3 = new Pose3d(pos);
        Logger.recordOutput("3DField/Chassis", pos3);
        // System.out.println(elevator.getPosition());
        Logger.recordOutput(
                "3DField/ElevatorMiddle",
                pos3.plus(new Transform3d(0, 0, elevator.getPosition() / 2, new Rotation3d())));

        Logger.recordOutput(
                "3DField/ElevatorInner", pos3.plus(new Transform3d(0, 0, elevator.getPosition(), new Rotation3d())));
    }
}
