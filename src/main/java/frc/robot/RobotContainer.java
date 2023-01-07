package frc.robot;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.fasterxml.jackson.databind.deser.std.FromStringDeserializer;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.ModuleTest;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.DriveSpinnyTest;
import frc.robot.drive.commands.DriveTurnSpinnyTest;
import frc.robot.drive.commands.ModuleTestCommand;
import frc.robot.drive.commands.TestModuleAngleCommand;
import frc.robot.drive.commands.TurnSpinnyTest;

import static frc.robot.Constants.*;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;
    private SwerveDrive swerve;

    public RobotContainer() {
        driver = new Joystick(DRIVER_PORT);

        navx = new AHRS(NAVX_PORT);

        // backLeft = new SwerveModule(
        //     Constants.BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_TURN_MOTOR_ID, BACK_LEFT_ENCODER_ID, BACK_LEFT_ANGLE_OFFSET);

        // moduleTest = new ModuleTest(backLeft, FRONT_RIGHT_POSITION);
        swerve = new SwerveDrive(navx);

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new DriveCommand(driver, swerve));
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 1).whenPressed(new InstantCommand(swerve::ResetHeading));
    }

    public void periodic() {
        swerve.updateSmartDash();
    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
