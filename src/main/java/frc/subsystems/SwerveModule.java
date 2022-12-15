package frc.subsystems;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap.DriveConstants;
import frc.robot.RobotMap.ModuleConstants;

public class SwerveModule {

    public final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    public double targetAngle, targetSpeed;

    // Instatiate new module with given ports and inversions
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed) {

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ROTATIONS_TO_METERS);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_RPM_TO_METERS_PER_SEC);
        turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ROTATIONS_TO_RAD);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_RPM_TO_RAD_PER_SEC);

        turningPidController = new PIDController(ModuleConstants.P_TURNING, 0, 0);

        // Let the controller know it's a circle and going past pi loops to -pi
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // At boot reset relative encoders to absolute
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    // Radians that the module is at, from 0 to 2pi
    public double getAbsoluteEncoderRad() {
        double angle = Math.toRadians(absoluteEncoder.getAbsolutePosition());
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    // Set drive encoder to 0 and turning encoder to match absolute
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        // If the speed is 0 (basically if the driver isn't touching joystick) don't snap motors to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Optimize to see if turning to opposite angle and running backwards is faster
        state = SwerveModuleState.optimize(state, getState().angle);

        // Set motors, using the turning pid controller for that motor
        targetAngle = state.angle.getDegrees();
        targetSpeed = state.speedMetersPerSecond;

        driveMotor.set(targetSpeed / DriveConstants.MAX_METERS_PER_SEC);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
