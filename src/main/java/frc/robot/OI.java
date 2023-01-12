package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.commands.TeleOpDrive;
import frc.commands.Test;
import frc.autos.TestAuto;
import frc.commands.AprilTagPoseEstimisation;
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

        // Press right bumper -> zero gyro heading
        driveRightBumper.onTrue(new InstantCommand(()->drivetrain.zeroHeading()));

        vision.setDefaultCommand(new AprilTagPoseEstimisation(vision, drivetrain));
    }

    /**
     * Initialize JoystickButtons and Controllers
     */
    private void initControllers() {
        driveController = new XboxController(0);
        driveRightBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    }

    // Return autocommand
    public Command getAutoCommand() {
        SequentialCommandGroup testAuto = new TestAuto(drivetrain);

        return testAuto;
    }

}