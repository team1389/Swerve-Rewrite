package frc.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.management.AttributeList;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("Microsoft Lifecam HD-3000");
    public PhotonPipelineResult result;
    public PhotonTrackedTarget bestTarget;
    public List<PhotonTrackedTarget> targets;
    public double fieldLength = 10; // meters
    public double fieldWidth = 10;

    final AprilTag tag01 =
            new AprilTag(01,
                        new Pose3d(new Pose2d(0.0, fieldWidth / 2.0, Rotation2d.fromDegrees(0.0))));
    List<AprilTag> atList = new ArrayList<AprilTag>();   
    AprilTagFieldLayout aprilTagFieldLayout;   

    Transform3d robotToCamTransformation = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));//Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    boolean hasTarget = false;

    ArrayList<Pair<PhotonCamera, Transform3d>> camList;
    RobotPoseEstimator robotPoseEstimator;
    

    // Instatiate new module with given ports and inversions
    public Vision() {
        getPipelineResult();

        atList.add(tag01);
        aprilTagFieldLayout = new AprilTagFieldLayout(atList, fieldLength, fieldWidth);

        camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCamTransformation));
        
        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
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

    /**
    * @param estimatedRobotPose The current best guess at robot pose. Pass the projected pose from the swerve odometry
    * @return A pair of the fused camera observations to a single Pose2d on the field, and the time of the observation. 
    *            Assumes a planar field and the robot is always firmly on the ground.
    *
    */
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
            return new Pair<Pose2d, Double>(
                    result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        } 
    }

    /**
     * Estimates the pose of the robot in the field coordinate system, given the id of the fiducial, the robot relative to the
     * camera, and the target relative to the camera.
     * @param tagPose Pose3d the field relative pose of the target
     * @param robotToCamera Transform3d of the robot relative to the camera. Origin of the robot is defined as the center.
     * @param cameraToTarget Transform3d of the target relative to the camera, returned by PhotonVision
     * @return Pose Robot position relative to the field.
    */


    public Pose3d getFieldToRobot(Pose3d tagPose, Transform3d robotToCamera, Transform3d cameraToTarget) {
 	    return tagPose.plus(cameraToTarget.inverse()).plus(robotToCamera.inverse()); 
    }


    public void stop() {
       
    }

    
}
