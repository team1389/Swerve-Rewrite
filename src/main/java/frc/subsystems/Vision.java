package frc.subsystems;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
    public PhotonCamera camera = new PhotonCamera("Microsoft Lifecam HD-3000");
    public double fieldLength = 10; // in meters
    public double fieldWidth = 10;
    public AprilTagFieldLayout aprilTagFieldLayout;
    // Cam mounted facing left, 0.197 meters behind center, 0.368 meters left of
    // center, 0.235 meters above center
    public final Transform3d robotToCamTransformation = new Transform3d(new Translation3d(-0.197, -0.368, 0.235),
            new Rotation3d(0, 0, 90));
    ArrayList<Pair<PhotonCamera, Transform3d>> camList;
    private RobotPoseEstimator robotPoseEstimator;

    public Vision() {

        try {
            aprilTagFieldLayout = new AprilTagFieldLayout("2023-chargedup.json");
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return;
        }

        camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCamTransformation));

        robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY,
                camList);
    }

    public void update() {
        var posePair = robotPoseEstimator.update();
        if (posePair.isEmpty()) {
            return;
        }

        // pose3d, may be null
        var pose = posePair.get().getFirst();
        // convert to seconds
        var latency = posePair.get().getSecond() / 1000;
        if (pose == null) {
            return; // no pose, we can do nothing
        }

        double time = System.currentTimeMillis() / 1000;
        var targetTime = time - latency;

        // TODO

    }
}
