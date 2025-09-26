package frc.robot.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.units.Units.Radians;

public class CameraIOPhotonCamera extends CameraIO {
    private PhotonCamera cam;
    private PhotonCameraSim sim;

    private Transform3d robotToCamera;

    private int currentIndex;

    public CameraIOPhotonCamera(String name, String logPath, Transform3d robotToCamera) {
        super(name, logPath);
        cam = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void update() {
        inputs.connected = cam.isConnected();

        var unreadResults = cam.getAllUnreadResults();
        for (PhotonPipelineResult res : unreadResults) {
            if (inputs.measurements < 64) {
                PhotonTrackedTarget target = res.getBestTarget();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                Pose3d aprilTagLocation =
                        Constants.layout.getTagPose(target.fiducialId).get();
                Pose3d estimatedRobotPose =
                        aprilTagLocation.plus(bestCameraToTarget.inverse()).plus(robotToCamera.inverse());
                inputs.poseXMeters[inputs.measurements] = estimatedRobotPose.getX();
                inputs.poseYMeters[inputs.measurements] = estimatedRobotPose.getY();
                inputs.poseRotationRad[inputs.measurements] =
                        estimatedRobotPose.getRotation().getMeasureZ().in(Radians);
                inputs.poseTimestamps[inputs.measurements] = Utils.fpgaToCurrentTime(res.getTimestampSeconds());
                inputs.measurements++;
            }
        }
    }

    @Override
    public void clearMeasurements() {
        inputs.measurements = 0;
    }
}
