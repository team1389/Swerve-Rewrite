package frc.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Vision;

public class AprilTagTracking extends CommandBase {
    public Vision vision;
    public PhotonTrackedTarget target;

    
    public AprilTagTracking(Vision vision) {
        this.vision = vision;
        addRequirements(vision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // 1. 
        vision.getPipelineResult();
        if(vision.hasTarget()) {
            target = vision.getTarget();
            System.out.println(target.getFiducialId());
        }
             
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
