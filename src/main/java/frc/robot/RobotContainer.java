package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.drive.commands.DriveCommand;

import static frc.robot.Constants.*;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;
    private SwerveDrive swerve;

    private SwerveModule frontLeft;

    public RobotContainer() {
        driver = new Joystick(DRIVER_PORT);

        frontLeft = new SwerveModule(
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_TURN_MOTOR_ID,
                FRONT_LEFT_ENCODER_ID,
                Rotation2d.fromDegrees(FRONT_LEFT_ANGLE_OFFSET));

        navx = new AHRS(NAVX_PORT);
        // swerve = new SwerveDrive(navx);

        // configureDefaultCommands();
        // configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new DriveCommand(driver, swerve));
    }

    private void configureButtonBindings() {

    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
