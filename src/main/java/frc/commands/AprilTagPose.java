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

public class AprilTagPose extends CommandBase {
    public Vision vision;
    public Drivetrain drivetrain;
    public PhotonTrackedTarget target;

    Pose3d tag01Pose;
    Pose3d fieldRelative3dPose;
    Pose2d fieldRelative2dPose;

    double xTranslation;
    double yTranslation;
    double angle;


    Transform3d robotToCamera;
    Transform3d cameraToTarget;

    
    public AprilTagPose(Vision vision, Drivetrain drivetrain) {
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
    
        vision.getPipelineResult();
        if(vision.hasTargets()) {
            target = vision.getTarget();
            cameraToTarget = vision.getCameraToTarget();

            fieldRelative3dPose = vision.getFieldToRobot(tag01Pose, robotToCamera, cameraToTarget);
            xTranslation = fieldRelative3dPose.getX();
            yTranslation = fieldRelative3dPose.getY();
            angle = fieldRelative3dPose.getRotation().getAngle();

            
            fieldRelative2dPose = new Pose2d(new Translation2d(xTranslation, yTranslation), new Rotation2d(angle));

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
