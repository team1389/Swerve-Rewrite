package frc.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.Pair;
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

    private ArrayList<Pair<Pose2d, Long>> positions = new ArrayList<>();

    private final Field2d field = new Field2d();

    // We want to reset gyro on boot, but the gyro takes a bit to start, so wait one
    // sec then do it (in seperate thread)
    public Drivetrain() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        SmartDashboard.putData("Field", field);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    // Returns degrees from -180 to 180
    public double getHeading() {
        return gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Position of the robot
    public Pose2d getOdometryPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void setFieldPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    @Override
    public void periodic() {
        positions.add(Pair.of(odometer.getPoseMeters(), System.currentTimeMillis()));
        odometer.update(getRotation2d(), getModulePositions());
        setFieldPose(odometer.getPoseMeters());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getOdometryPose().getTranslation().toString());
        System.out.println(positions);
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

    public void updatePosistion(Pose2d knownLocation, long timeDetected) {
        // find two points that are around the time
        Pair<Pose2d, Long> first = new Pair<>(null, null);
        Pair<Pose2d, Long> second = new Pair<>(null, null);

        for (int i = positions.size() - 1; i <= 0; i--) {
            var pos = positions.get(i);
            if (first.getSecond() == null || pos.getSecond() - timeDetected <= 0) {
                first = pos;
                second = positions.get(i - 1);
                break;
            }
        }
        var currentEstimatedPose = positions.get(positions.size() - 1).getFirst();

        if (first.getSecond() == null | second.getSecond() == null) {
            return;
        }

        // interoplate between points
        double timeBetween = first.getSecond() - second.getSecond();
        double time = first.getSecond() - timeDetected;
        double factor = time / timeBetween; // number between 0 and 1
        double inverseFactor = 1 - factor;
        var poseAtTime = add(first.getFirst().times(factor), second.getFirst().times(inverseFactor));
        
        // apply difference in location that occured b/c of latency
        var currentPose = knownLocation.plus(currentEstimatedPose.minus(poseAtTime));

        odometer.resetPosition(getRotation2d(), getModulePositions(), currentPose);
    }

    Pose2d add(Pose2d a, Pose2d b) {
        var transA = a.getTranslation();
        var transB = b.getTranslation();
        var transOut = transA.plus(transB);

        var rotA = a.getRotation();
        var rotB = b.getRotation();
        var rotOut = rotA.plus(rotB);
        return new Pose2d(transOut, rotOut);
    }
}
