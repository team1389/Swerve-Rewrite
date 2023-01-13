package frc.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;
import frc.subsystems.Vision;

public class AprilTagPoseEstimisation extends CommandBase {
    public Vision vision;
    public Drivetrain drivetrain;
    public PhotonTrackedTarget target;

    Pose3d tag01Pose;
    Pose3d fieldRelative3dPose;

    Pose2d fieldRelative2dPose;
    Pose2d estimatedFieldRelative2dPose;


    Pose2d drivetrainPose;

    double xTranslation;
    double yTranslation;
    double angle;


    Transform3d robotToCamera;
    Transform3d cameraToTarget;

    
    public AprilTagPoseEstimisation(Vision vision, Drivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        tag01Pose = vision.aprilTagFieldLayout.getTagPose(1).get();
        robotToCamera = vision.robotToCamTransformation;

    }

    @Override
    public void execute() {
        if(vision.hasTarget()) {
            cameraToTarget = vision.getCameraToTarget();
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
