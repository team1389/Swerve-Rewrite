package frc.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("Microsoft Lifecam HD-3000");
    public PhotonPipelineResult result;
    public PhotonTrackedTarget bestTarget;
    public List<PhotonTrackedTarget> targets;
    public double fieldLength = 10; // meters
    public double fieldWidth = 10;
    List<AprilTag> aprilTags;
    
    AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, fieldLength, fieldWidth);
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));//Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    // ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    // ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    // camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCam));
    // AprilTagPoseEstimator robotPoseEstimator = new AprilTagPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    
    boolean hasTarget = false;

    // Instatiate new module with given ports and inversions
    public Vision() {
        getPipelineResult();
    }

    public void getPipelineResult() {
        result = camera.getLatestResult();
    }

    public PhotonTrackedTarget getTarget() {
        getPipelineResult();
        if(hasTargets()) {
            bestTarget = result.getBestTarget();
            targets = result.getTargets();
        }
        return bestTarget;
    }

    public int getFiducialID() {
        getTarget();

        if(result.hasTargets()) {
            return bestTarget.getFiducialId();
        }
        return 0;
    
    }

    public boolean hasTargets() {
        getPipelineResult();
        return result.hasTargets();
    }

    public void stop() {
       
    }
}
