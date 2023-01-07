package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.ModuleTest;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.VelocityTest;
import frc.robot.drive.commands.VoltageTest;

import static frc.robot.Constants.*;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;
    private SwerveDrive swerve;

    private SwerveModule frontLeft;
    private ModuleTest test;

    public RobotContainer() {
        driver = new Joystick(DRIVER_PORT);

        navx = new AHRS(NAVX_PORT);

        // frontLeft = new SwerveModule(FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_TURN_MOTOR_ID, FRONT_LEFT_ENCODER_ID, FRONT_LEFT_ANGLE_OFFSET);

        // test = new ModuleTest(frontLeft, FRONT_LEFT_POSITION);

        swerve = new SwerveDrive(navx);

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new DriveCommand(driver, swerve));
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 1).whenPressed(new InstantCommand(swerve::ResetHeading));

        // new JoystickButton(driver, 2).whileHeld(new VelocityTest(test, 0.5));
        // new JoystickButton(driver, 3).whileHeld(new VoltageTest(test, 1));
        // SmartDashboard.putNumber("volts", 0);
    }

    public void periodic() {
        swerve.updateSmartDash();
        // SmartDashboard.putNumber("Module Velocity", test.getModuleVelocityMPS());
    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
