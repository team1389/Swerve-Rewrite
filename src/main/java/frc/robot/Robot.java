package frc.robot;


import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.SwerveTelemetry;

/**
 * Don't change the name of this class since the VM is set up to run this
 */
public class Robot extends TimedRobot {


    private OI oi;


    @Override
    public void robotInit() {
        
        oi = new OI();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }


    @Override
    public void autonomousInit() {
        //Example of setting auto: Scheduler.getInstance().add(YOUR AUTO);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        SwerveTelemetry frontLeftTelemetry = new SwerveTelemetry(oi.drivetrain.frontLeft);
        SwerveTelemetry backLeftTelemetry = new SwerveTelemetry(oi.drivetrain.backLeft);
        SwerveTelemetry frontRightTelemetry = new SwerveTelemetry(oi.drivetrain.frontRight);
        SwerveTelemetry backRightTelemetry = new SwerveTelemetry(oi.drivetrain.backRight);
        //SendableRegistry.add(frontLeftTelemetry, "Swerve");
        SendableRegistry.addLW(frontLeftTelemetry, "FL Swerve");
        SendableRegistry.addLW(backLeftTelemetry, "BL Swerve");
        SendableRegistry.addLW(frontRightTelemetry, "FR Swerve");
        SendableRegistry.addLW(backRightTelemetry, "BR Swerve");

        oi.drivetrain.frontLeft.resetEncoders();
        oi.drivetrain.backLeft.resetEncoders();
        oi.drivetrain.frontRight.resetEncoders();
        oi.drivetrain.backRight.resetEncoders();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}