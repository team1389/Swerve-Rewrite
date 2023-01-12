package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.commands.TeleOpDrive;
import frc.commands.Test;
import frc.commands.AprilTagPose;
import frc.subsystems.Drivetrain;
import frc.subsystems.Vision;

public class OI {

    public final Drivetrain drivetrain = new Drivetrain();
    public final Vision vision = new Vision();

    private XboxController driveController;
    private Trigger driveRightBumper;

    public OI() {
        initControllers();
        
        // Cool new way to make a drive command by passing in Suppliers for the joysticks
        drivetrain.setDefaultCommand(new TeleOpDrive(
            drivetrain,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> -driveController.getRightY(),
            () -> !driveController.getLeftBumper()) // By default be in field oriented
        );

        driveRightBumper.onTrue(new InstantCommand(()->drivetrain.zeroHeading()));
        
        vision.setDefaultCommand(new AprilTagPose(vision, drivetrain));
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        driveRightBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    }


}