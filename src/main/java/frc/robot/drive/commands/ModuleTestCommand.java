package frc.robot.drive.commands;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.drive.ModuleTest;

public class ModuleTestCommand extends CommandBase {

    private Joystick driver;
    private ModuleTest test;

    public ModuleTestCommand(Joystick driver, ModuleTest test) {
        this.driver = driver;
        this.test = test;
    }

    @Override
    public void execute() {
        double xSpeed = calculateAxis(driver.getY(), Constants.AXIS_DEADBAND, Constants.MAX_TRANSLATIONAL_VELOCITY);
        double ySpeed = calculateAxis(driver.getX(), Constants.AXIS_DEADBAND, Constants.MAX_TRANSLATIONAL_VELOCITY);
        double thetaSpeed = calculateAxis(driver.getZ(), Constants.AXIS_DEADBAND, Constants.MAX_ROTATIONAL_VELOCITY);

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        test.setOpenLoop(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        test.setOpenLoop(new ChassisSpeeds());
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
