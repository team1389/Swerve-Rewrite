package frc.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Drivetrain;
import frc.subsystems.Vision;

public class AprilTagPoseEstimisation extends CommandBase {
    //Define subsystems, pose (cordinates and rotation), and transform (pose vector) objects
    public Vision vision;
    public Drivetrain drivetrain;

    Pose2d fieldRelative2dPose;
    Pose2d estimatedFieldRelative2dPose;

    Pose2d drivetrainPose;

    Transform3d robotToCamera;
    Transform3d cameraToTarget;

    public AprilTagPoseEstimisation(Vision vision, Drivetrain drivetrain) {
        //Initialize subsystem and transformation
        this.vision = vision;
        this.drivetrain = drivetrain;
        robotToCamera = vision.robotToCamTransformation;

        addRequirements(vision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //Check if PhotonVision has a tracked target.
        if(vision.hasTarget()) {
            drivetrainPose = drivetrain.getPose();

            estimatedFieldRelative2dPose = vision.getEstimatedGlobalPose(drivetrainPose).getFirst();

            if(estimatedFieldRelative2dPose != null) {
                fieldRelative2dPose = estimatedFieldRelative2dPose;
            }
            else {
                fieldRelative2dPose = new Pose2d();
            }

            SmartDashboard.putString("Drivetrain 2D POSE", drivetrainPose.toString());
            SmartDashboard.putString("AT ESTIMATED 2D POSE", fieldRelative2dPose.toString());

            drivetrain.setFieldPose(fieldRelative2dPose);
            
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
