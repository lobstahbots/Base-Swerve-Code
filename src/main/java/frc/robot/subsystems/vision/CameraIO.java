package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public Pose3d bestEstimatedPose = null;
        public double bestReprojError = 0.0;

        public Pose3d altEstimatedPose = null;
        public double altReprojError = 0.0;

        public double ambiguity = 0.0;
        
        public double estimatedPoseTimestamp = 0.0;

        public int[] visibleFiducialIDs = new int[] {};

        public double totalArea = 0.0;
    }

    public List<PhotonTrackedTarget> getTrackedTargets();

    public void updateInputs(CameraIOInputs inputs, Pose3d robotPose);
}
