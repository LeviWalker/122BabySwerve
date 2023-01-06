package frc.robot;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.DriveSpinnyTest;
import frc.robot.drive.commands.DriveTurnSpinnyTest;
import frc.robot.drive.commands.TurnSpinnyTest;

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
                new Rotation2d(FRONT_LEFT_ANGLE_OFFSET));

        navx = new AHRS(NAVX_PORT);
        // swerve = new SwerveDrive(navx);

        // configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new DriveCommand(driver, swerve));
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 3).whileHeld(new DriveSpinnyTest(frontLeft));
        new JoystickButton(driver, 4).whileHeld(new TurnSpinnyTest(frontLeft));
        new JoystickButton(driver, 2).whileHeld(new DriveTurnSpinnyTest(frontLeft));
    }

    public void periodic() {

    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
