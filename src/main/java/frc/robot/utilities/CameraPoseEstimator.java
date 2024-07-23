package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
/**
 * This class was created as a way to filter out images where
 * the ambiguity was too high.
 * It is used instead of the super class PhotonPoseEstimator
 */
public class CameraPoseEstimator extends PhotonPoseEstimator {
    
    /**
     * Any apriltag with an ambiguity higher than this number will be ignored.`
     */
    public final static double MAX_AMBIGUITY = 0.2;
    //private String cameraId;

    public CameraPoseEstimator(String cameraId, AprilTagFieldLayout fieldTags, PoseStrategy strategy, PhotonCamera camera,
            Transform3d robotToCamera) {
        super(fieldTags, strategy, camera, robotToCamera);
        //this.cameraId = cameraId;
    }

    @Override
    public Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult) {    
        List<PhotonTrackedTarget> targets = cameraResult.getTargets();
        ArrayList<PhotonTrackedTarget> result = new ArrayList<>();
        for (int i=targets.size()-1;i>=0;i--) {
            PhotonTrackedTarget target = targets.get(i);
            if (target.getPoseAmbiguity() > MAX_AMBIGUITY) {            
                // logf("Camera Id = %s removing target id %d with ambiguity %.2f\n", cameraId, target.getFiducialId(), target.getPoseAmbiguity());
            } else {
                result.add(target);
                // logf("Camera Id = %s Not removing target id %d with ambiguity %.2f\n", cameraId, target.getFiducialId(), target.getPoseAmbiguity());
            }

        }
        PhotonPipelineResult photonPipelineResult = new PhotonPipelineResult(cameraResult.getLatencyMillis(), result);
        photonPipelineResult.setTimestampSeconds(cameraResult.getTimestampSeconds());
        return super.update(photonPipelineResult);
    }
}
