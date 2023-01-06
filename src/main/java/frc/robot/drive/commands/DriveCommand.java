package frc.robot.drive.commands;

import frc.robot.MathUtil;
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
        double xSpeed = MathUtil.calculateAxis(driver.getY(), Constants.AXIS_DEADBAND, Constants.MAX_TRANSLATIONAL_VELOCITY);
        double ySpeed = MathUtil.calculateAxis(driver.getX(), Constants.AXIS_DEADBAND, Constants.MAX_TRANSLATIONAL_VELOCITY);
        double thetaSpeed = MathUtil.calculateAxis(driver.getZ(), Constants.AXIS_DEADBAND, Constants.MAX_ROTATIONAL_VELOCITY);

        ChassisSpeeds speeds;

        if (Constants.FIELD_RELATIVE) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, swerve.getHeading());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        }

        swerve.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }
}
