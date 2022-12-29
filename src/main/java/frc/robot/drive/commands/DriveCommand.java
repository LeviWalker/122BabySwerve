package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.drive.SwerveDrive;

public class DriveCommand extends CommandBase {

    private Joystick driver;
    private SwerveDrive swerve;

    public DriveCommand(Joystick driver, SwerveDrive swerve) {
        this.driver = driver;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xSpeed = calculateAxis(driver.getY(), Constants.AXIS_DEADBAND, Constants.MAX_TRANSLATIONAL_VELOCITY);
        double ySpeed = calculateAxis(driver.getX(), Constants.AXIS_DEADBAND, Constants.MAX_TRANSLATIONAL_VELOCITY);
        double thetaSpeed = calculateAxis(driver.getZ(), Constants.AXIS_DEADBAND, Constants.MAX_ROTATIONAL_VELOCITY);

        ChassisSpeeds speeds;

        if (Constants.FIELD_RELATIVE) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, swerve.getHeading());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        }

        swerve.drive(speeds);
    }

    private double calculateAxis(double axis, double deadband, double scalar) {

        double res;

        if (Math.abs(axis) > deadband) {
            if (axis > 0.0) {
                res = (axis - deadband) / (1.0 - deadband);
            } else {
                res = (axis + deadband) / (1.0 - deadband);
            }
        } else {
            res = 0.0;
        }

        return res * scalar;
    }
}
