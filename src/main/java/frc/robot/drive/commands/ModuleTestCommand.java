package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.MathUtil;
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
        double xSpeed = MathUtil.calculateAxis(driver.getY(), Constants.AXIS_DEADBAND, Constants.MAX_TRANSLATIONAL_VELOCITY);
        double ySpeed = MathUtil.calculateAxis(driver.getX(), Constants.AXIS_DEADBAND, Constants.MAX_TRANSLATIONAL_VELOCITY);
        double thetaSpeed = MathUtil.calculateAxis(driver.getZ(), Constants.AXIS_DEADBAND, Constants.MAX_ROTATIONAL_VELOCITY);

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        test.setOpenLoop(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        test.setOpenLoop(new ChassisSpeeds());
    }
}
