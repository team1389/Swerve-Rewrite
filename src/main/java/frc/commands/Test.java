package frc.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Drivetrain;
import frc.robot.RobotMap.DriveConstants;

public class Test extends CommandBase{
    private final Drivetrain drivetrain;
    private final double xSpeed, ySpeed, turnSpeed;
    
    public Test(Drivetrain drivetrain, double xSpeed, double ySpeed, double turnSpeed) {
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.turnSpeed = turnSpeed;
    }

    @Override
    public void execute() {
        // Set drivetrain to desired speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

        // Convert chassis speeds to individual module states and set wheels
        SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);
        drivetrain.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}
