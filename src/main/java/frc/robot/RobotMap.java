package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * define Hardware Ports in here
 * also swerve constants
 */
public class RobotMap {

    // Constants that are the same for the 4 swerve modules
    public static final class ModuleConstants {
        // Note: these are for the drive and turning motors
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
        public static final double DRIVE_GEAR_RATIO = 1 / 5.25;
        public static final double TURN_GEAR_RATIO = 1 / (5.33 * 7);
        public static final double DRIVE_ROTATIONS_TO_METERS = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ROTATIONS_TO_RAD = TURN_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_RPM_TO_METERS_PER_SEC = DRIVE_ROTATIONS_TO_METERS / 60;
        public static final double TURNING_RPM_TO_RAD_PER_SEC = TURNING_ROTATIONS_TO_RAD / 60;
        public static final double P_TURNING = 0.5; // PID constant TODO: Tune this
    }

    // All the overall constants for the drivetrain
    public static final class DriveConstants {

        // Distance between right and left wheels (meters)
        public static final double ROBOT_WIDTH = 0.562991;

        // Distance between front and back wheels (meters)
        public static final double ROBOT_LENGTH = 0.562991;

        // Note positive x is forward
        // Wheel order: FR, FL, BR, BL
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
                new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
                new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2));

        // FL is front left, BR is back right, etc.
        public static final int FL_DRIVE_PORT = 3;
        public static final int BL_DRIVE_PORT = 7;
        public static final int FR_DRIVE_PORT = 5;
        public static final int BR_DRIVE_PORT = 9;

        public static final int FL_TURN_PORT = 4;
        public static final int BL_TURN_PORT = 8;
        public static final int FR_TURN_PORT = 6;
        public static final int BR_TURN_PORT = 10;

        // Sometimes encoders are mounted backwards based on robot design, this fixes
        // that although it's not a thing on stargazer
        public static final boolean FL_TURN_REVERSED = true;
        public static final boolean BL_TURN_REVERSED = true;
        public static final boolean FR_TURN_REVERSED = true;
        public static final boolean BR_TURN_REVERSED = true;

        public static final boolean FL_DRIVE_REVERSED = false;
        public static final boolean BL_DRIVE_REVERSED = false;
        public static final boolean FR_DRIVE_REVERSED = false;
        public static final boolean BR_DRIVE_REVERSED = false;

        // Absolute encoder ports
        public static final int FL_ABS_PORT = 11;
        public static final int BL_ABS_PORT = 13;
        public static final int FR_ABS_PORT = 12;
        public static final int BR_ABS_PORT = 14;

        public static final boolean FL_ABS_REVERSED = false;
        public static final boolean BL_ABS_REVERSED = false;
        public static final boolean FR_ABS_REVERSED = false;
        public static final boolean BR_ABS_REVERSED = false;

        // The physical max if motors go full speed
        public static final double MAX_METERS_PER_SEC = 2.177; // m/s
        public static final double MAX_RADIANS_PER_SEC = 0.785; // rad/s

        public static final double MAX_LINEAR_ACCEL = 0.381; // m/s/s
        public static final double MAX_ANGULAR_ACCEL = 0.224; // rad/s/s
    }
}