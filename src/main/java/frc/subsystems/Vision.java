package frc.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("photonvision");
    public PhotonPipelineResult result;
    public PhotonTrackedTarget target;

    
    boolean hasTarget = false;

    // Instatiate new module with given ports and inversions
    public Vision() {
        getResult();
    }

    public PhotonPipelineResult getResult() {
        result = camera.getLatestResult();
        return result;
    }

    public PhotonTrackedTarget getTarget() {
        getResult();
        if(result.hasTargets()) {
            target = result.getBestTarget();
            return target;
        }
        return null;
    }

    public int getFiducialID() {
        getTarget();

        if(result.hasTargets()) {
            return target.getFiducialId();
        }
        return 0;
    
    }


    public boolean hasTargets() {
        getResult();
        return result.hasTargets();
    }



    public void stop() {
       
    }
}
