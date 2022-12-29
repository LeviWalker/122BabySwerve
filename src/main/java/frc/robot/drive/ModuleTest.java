package frc.robot.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleTest {
    private SwerveModule module;
    private SwerveDriveKinematics kinematics;

    public ModuleTest(SwerveModule module, Translation2d position) {
        this.module = module;
        this.kinematics = new SwerveDriveKinematics(position);
    }

    public void setOpenLoop(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        if (states.length != 1) throw new IllegalStateException("there should be only one state");

        module.setDesiredState(states[0], true);
    }

    public void setClosedLoop(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        if (states.length != 1) throw new IllegalStateException("there should be only one state");

        module.setDesiredState(states[0], false);
    }
}
