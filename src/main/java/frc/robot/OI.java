package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.commands.TeleOpDrive;
import frc.commands.Test;
import frc.subsystems.Drivetrain;

public class OI {

    public final Drivetrain drivetrain = new Drivetrain();

    private XboxController driveController;

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
        //drivetrain.setDefaultCommand(new Test(drivetrain, 0.5, 0, 0));
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
    }


}