package frc.subsystems;

import java.io.IOException;
import java.lang.reflect.Field;
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

import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.FieldConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-300");
    public PhotonPipelineResult result;
    public PhotonTrackedTarget bestTarget;
    public List<PhotonTrackedTarget> targets;

    

    final AprilTag tag01 =
            new AprilTag(01,
                        new Pose3d(new Pose2d(0.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(0.0))));
    List<AprilTag> atList = new ArrayList<AprilTag>();   
    public AprilTagFieldLayout aprilTagFieldLayout;   

    public final Transform3d robotToCamTransformation = new Transform3d(new Translation3d(0, -0.368, 0.35), new Rotation3d(0,0,1.571));//Cam mounted facing left, 0.197 meters behind center, 0.368 meters left of center, 0.235 meters above center

    boolean hasTarget = false;

    ArrayList<Pair<PhotonCamera, Transform3d>> camList;
    public RobotPoseEstimator robotPoseEstimator;
    
    // Instatiate new module with given ports and inversions
    public Vision() {
        getPipelineResult();

        atList.add(tag01);
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout("2023-chargedup.json");
        } catch (IOException e) {
            aprilTagFieldLayout = new AprilTagFieldLayout(atList, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);
        }

        camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCamTransformation));
        
        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);
    }

    public void getPipelineResult() {
        result = camera.getLatestResult();
    }

    public PhotonTrackedTarget getTarget() {
        if(hasTarget()) {
            bestTarget = result.getBestTarget();
            targets = result.getTargets();
        }
        return bestTarget;
    }

    public String getFiducialID() {
        if(hasTarget()) {
            return targets.toString();
        }
        return "";
    
    }

    public Transform3d getCameraToTarget() {
        if(hasTarget()) {
            return bestTarget.getBestCameraToTarget();
        }
        return new Transform3d();
    }

    public boolean hasTarget() {
        hasTarget = result.hasTargets();
        SmartDashboard.putBoolean("hasTarget", hasTarget);
        return hasTarget;
    }

    public void periodic() {
        getTarget();
        hasTarget();
        robotPoseEstimator.update();
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
            System.out.println("RESULT IS NOT PRESENT");
            return new Pair<Pose2d, Double>(null, 0.0);
        } 
    }

    public void stop() {

    }

}
