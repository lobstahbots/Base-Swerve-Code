package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.VisionConstants;

public class CameraIOPhoton implements CameraIO {
    private final PhotonCamera camera;
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2024Crescendo);
    private final LobstahPoseEstimator poseEstimator;
    private LobstahEstimatedRobotPose estimatedPose = new LobstahEstimatedRobotPose(null, null, 0.0, 0.0, 0.0, 0.0,
            new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    private final Alert disconnectedAlert;

    public CameraIOPhoton(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new LobstahPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, robotToCamera);
        disconnectedAlert = new Alert(cameraName + " has disconnected.", AlertType.kError);
        disconnectedAlert.set(false);
    }

    public void updateInputs(CameraIOInputs inputs, Pose3d robotPose) {
        List<PhotonPipelineResult> poseResults = camera.getAllUnreadResults();
        if (poseResults.size() > 0) {
            Optional<LobstahEstimatedRobotPose> poseOptional = poseEstimator
                    .update(poseResults.get(poseResults.size() - 1));
            if (poseOptional.isPresent()) {
                estimatedPose = poseOptional.get();

                inputs.bestEstimatedPose = estimatedPose.bestEstimatedPose;
                inputs.altEstimatedPose = estimatedPose.alternateEstimatedPose;
                inputs.bestReprojError = estimatedPose.bestReprojError;
                inputs.altReprojError = estimatedPose.altReprojError;
                inputs.estimatedPoseTimestamp = estimatedPose.timestampSeconds;
                inputs.visibleFiducialIDs = estimatedPose.fiducialIDsUsed;
                inputs.totalArea = estimatedPose.totalArea;
                inputs.ambiguity = estimatedPose.multiTagAmbiguity;
            } else {
                inputs.bestEstimatedPose = null;
                inputs.altEstimatedPose = null;
                inputs.bestReprojError = 0.0;
                inputs.visibleFiducialIDs = new int[0];
                inputs.totalArea = 0;
                inputs.ambiguity = 0;
                inputs.altReprojError = 0;
            }
        }
        disconnectedAlert.set(!camera.isConnected());
    }

    public List<PhotonTrackedTarget> getTrackedTargets() {
        return estimatedPose.targetsUsed;
    }

    @AutoLogOutput
    public Pose3d[] getTagPoses() {
        var targets = getTrackedTargets();
        Pose3d[] tagPoses = new Pose3d[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            tagPoses[i] = aprilTagFieldLayout.getTagPose(targets.get(i).getFiducialId()).get();
        }
        return tagPoses;
    }
}
