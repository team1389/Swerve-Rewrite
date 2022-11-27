package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.commands.TeleOpDrive;
import frc.subsystems.Drivetrain;

public class OI {

    private final Drivetrain drivetrain = new Drivetrain();

    private XboxController driveController, manipController;

    public OI() {
        initControllers();
        
        // Cool new way to make a drive command by passing in Suppliers for the joysticks
        drivetrain.setDefaultCommand(new TeleOpDrive(
            drivetrain,
            () -> driveController.getLeftX(),
            () -> -driveController.getLeftY(),
            () -> driveController.getRightX(),
            () -> !driveController.getLeftBumper()) // By default be in field oriented
        );
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        manipController = new XboxController(1);
    }


}