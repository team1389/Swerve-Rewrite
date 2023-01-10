package frc.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("Microsoft Lifecam HD-3000");
    public PhotonPipelineResult result;
    public PhotonTrackedTarget bestTarget;
    public List<PhotonTrackedTarget> targets;
    public AprilTagFieldLayout aprilTagFieldLayout = new ApriltagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));


    
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
            targets = result.getTargets()
]        }
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
