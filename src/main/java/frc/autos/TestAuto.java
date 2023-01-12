package frc.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(Drivetrain drivetrain) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.AUTO_MAX_METERS_PER_SEC,
            DriveConstants.MAX_LINEAR_ACCEL)
            .setKinematics(DriveConstants.driveKinematics);

        // 2. Generate trajectory (manually for now, add pathplanner later)
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     List.of(
        //         new Translation2d(1, 0),
        //         new Translation2d(1, -1)),
        //     new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        //     trajectoryConfig);
        // Basic drive 2 meters forward:
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 0)),
            new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.P_AUTO_X, 0, 0);
        PIDController yController = new PIDController(AutoConstants.P_AUTO_Y, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.P_AUTO_THETA, 0, 0, AutoConstants.THETA_CONTROL_PROFILE);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose,
            DriveConstants.driveKinematics,
            xController,
            yController,
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        // 5. Add some init and wrap-up, add everything
        addCommands(
            new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> drivetrain.stopModules()));
    }
}
