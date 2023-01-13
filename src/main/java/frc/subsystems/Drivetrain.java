package frc.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.AutoConstants;
import frc.robot.RobotMap.DriveConstants;

public class Drivetrain extends SubsystemBase {
    public final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FL_DRIVE_PORT,
        DriveConstants.FL_TURN_PORT,
        DriveConstants.FL_DRIVE_REVERSED,
        DriveConstants.FL_TURN_REVERSED,
        DriveConstants.FL_ABS_PORT,
        DriveConstants.FR_ABS_REVERSED);

    public final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FR_DRIVE_PORT,
        DriveConstants.FR_TURN_PORT,
        DriveConstants.FR_DRIVE_REVERSED,
        DriveConstants.FR_TURN_REVERSED,
        DriveConstants.FR_ABS_PORT,
        DriveConstants.FR_ABS_REVERSED);

    public final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BL_DRIVE_PORT,
        DriveConstants.BL_TURN_PORT,
        DriveConstants.BL_DRIVE_REVERSED,
        DriveConstants.BL_TURN_REVERSED,
        DriveConstants.BL_ABS_PORT,
        DriveConstants.BL_ABS_REVERSED);

    public final SwerveModule backRight = new SwerveModule(
        DriveConstants.BR_DRIVE_PORT,
        DriveConstants.BR_TURN_PORT,
        DriveConstants.BR_DRIVE_REVERSED,
        DriveConstants.BR_TURN_REVERSED,
        DriveConstants.BR_ABS_PORT,
        DriveConstants.BR_ABS_REVERSED);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.driveKinematics,
            new Rotation2d(0), getModulePositions());
    
    private final Field2d m_field = new Field2d();


    // We want to reset gyro on boot, but the gyro takes a bit to start, so wait one sec then do it (in seperate thread)
    public Drivetrain() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        SmartDashboard.putData("Field", m_field);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    // Returns degrees from -180 to 180
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Position of the robot
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void setFieldPose(Pose2d pose) {
        m_field.setRobotPose(pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Wheel order: FR, FL, BR, BL
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontRight.getPosition(),
            frontLeft.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Normalize to within robot max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_METERS_PER_SEC);
        
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
        
    }


    // Return a command to follow given pathplannertrajectory
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
         // Define PID controllers for tracking trajectory
         PIDController xController = new PIDController(AutoConstants.P_AUTO_X, 0, 0);
         PIDController yController = new PIDController(AutoConstants.P_AUTO_Y, 0, 0);
         PIDController thetaController = new PIDController(AutoConstants.P_AUTO_THETA, 0, 0);
         thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getInitialHolonomicPose());
            }
            }),
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, // Pose supplier
                DriveConstants.driveKinematics, 
                xController, // X controller
                yController, // Y controller
                thetaController, // Rotation controller
                this::setModuleStates, // Module states consumer
                this
            )
        );
    }

}
