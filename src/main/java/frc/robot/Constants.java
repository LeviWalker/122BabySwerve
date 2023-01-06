package frc.robot;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;

public class Constants {
    public static final int DRIVER_PORT = 0;
    public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;

    public static final ModuleConfiguration SDS_MODULE_DATA = SdsModuleConfigurations.MK4I_L2;

    public static final double DRIVE_TRACKWIDTH_METERS = 0;
    public static final double DRIVE_WHEELBASE_METERS = 0;

    public static final double AXIS_DEADBAND = 0.05;
    // rpm / (60 s / min) * (g * C m / rot) where g is the gear ratio and C is the curcumference of the 
    /**
     * Maximum translational velocity of the robot in meters per second
     */
    public static final double MAX_TRANSLATIONAL_VELOCITY = (6380.0 / 60) * SDS_MODULE_DATA.getWheelDiameter() * Math.PI
            * SDS_MODULE_DATA.getDriveReduction();
    public static final double MAX_ROTATIONAL_VELOCITY = MAX_TRANSLATIONAL_VELOCITY
            / Math.hypot(DRIVE_TRACKWIDTH_METERS / 2, DRIVE_WHEELBASE_METERS / 2);
    public static final boolean FIELD_RELATIVE = false;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 10;
    public static final int FRONT_LEFT_TURN_MOTOR_ID = 11;
    public static final int FRONT_LEFT_ENCODER_ID = 12;
    public static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(323.9 - 90 + 48);

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 20;
    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 21;
    public static final int FRONT_RIGHT_ENCODER_ID = 22;
    public static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(145.8 - 90 + 82);

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 30;
    public static final int BACK_LEFT_TURN_MOTOR_ID = 31;
    public static final int BACK_LEFT_ENCODER_ID = 32;
    public static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(72.5 - 90 + 115);

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 40;
    public static final int BACK_RIGHT_TURN_MOTOR_ID = 41;
    public static final int BACK_RIGHT_ENCODER_ID = 42;
    public static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(269.2 - 90 + 82);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVE_TRACKWIDTH_METERS / 2.0, DRIVE_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVE_TRACKWIDTH_METERS / 2.0, -DRIVE_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVE_TRACKWIDTH_METERS / 2.0, DRIVE_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVE_TRACKWIDTH_METERS / 2.0, -DRIVE_WHEELBASE_METERS / 2.0));
}
