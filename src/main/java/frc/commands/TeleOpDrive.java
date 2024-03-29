package frc.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap.DriveConstants;
import frc.subsystems.Drivetrain;

public class TeleOpDrive extends CommandBase {

    private final Drivetrain drivetrain;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double desiredAngle; // gyro value from getHeading() the robot wants to point at

    public TeleOpDrive(Drivetrain drivetrain,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Double> rightY,
            Supplier<Boolean> fieldOrientedFunction) {
        this.drivetrain = drivetrain;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        // A slew rate limiter caps the rate of change of the inputs, to make the robot drive much smoother
        this.xLimiter = new SlewRateLimiter(DriveConstants.MAX_LINEAR_ACCEL);
        this.yLimiter = new SlewRateLimiter(DriveConstants.MAX_LINEAR_ACCEL);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.MAX_ANGULAR_ACCEL);

        this.desiredAngle = drivetrain.getHeading();
        

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 0. push desired angle to smart dashboard
        SmartDashboard.putNumber("Desired Angle", desiredAngle);
        
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > 0.03 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.03 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > 0.03 ? turningSpeed : 0.0;

        // 3. Make the driving smoother using the slew limiter
        xSpeed = xLimiter.calculate(xSpeed * DriveConstants.MAX_METERS_PER_SEC);
        ySpeed = yLimiter.calculate(ySpeed * DriveConstants.MAX_METERS_PER_SEC);
        turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.MAX_RADIANS_PER_SEC);
        // xSpeed = xSpeed * DriveConstants.MAX_METERS_PER_SEC;
        // ySpeed = ySpeed * DriveConstants.MAX_METERS_PER_SEC;
        // turningSpeed = turningSpeed * DriveConstants.MAX_RADIANS_PER_SEC;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, drivetrain.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
        SmartDashboard.putNumber("FR target", moduleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("FL target", moduleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("FR target", moduleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("FL target", moduleStates[3].angle.getDegrees());

        // 6. Output all module states to wheels
        drivetrain.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
