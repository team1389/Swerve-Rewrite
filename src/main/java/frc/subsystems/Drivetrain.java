package frc.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        m_field.setRobotPose(odometer.getPoseMeters());
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
        
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
