package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {

        private AHRS navx;

        private ChassisSpeeds speeds;
        private SwerveDriveKinematics kinematics;
        private SwerveDriveOdometry odometry;

        private SwerveModule frontLeft, frontRight, backLeft, backRight;
        private SwerveModule[] modules;

        public SwerveDrive(AHRS navx) {
                this.navx = navx;
                this.navx.calibrate();

                this.speeds = new ChassisSpeeds();
                this.kinematics = new SwerveDriveKinematics(
                                // Front left
                                new Translation2d(Constants.DRIVE_TRACKWIDTH_METERS / 2.0,
                                                Constants.DRIVE_WHEELBASE_METERS / 2.0),
                                // Front right
                                new Translation2d(Constants.DRIVE_TRACKWIDTH_METERS / 2.0,
                                                -Constants.DRIVE_WHEELBASE_METERS / 2.0),
                                // Back left
                                new Translation2d(-Constants.DRIVE_TRACKWIDTH_METERS / 2.0,
                                                Constants.DRIVE_WHEELBASE_METERS / 2.0),
                                // Back right
                                new Translation2d(-Constants.DRIVE_TRACKWIDTH_METERS / 2.0,
                                                -Constants.DRIVE_WHEELBASE_METERS / 2.0));

                this.frontLeft = new SwerveModule(
                                Constants.FRONT_LEFT_DRIVE_MOTOR_ID,
                                Constants.FRONT_LEFT_TURN_MOTOR_ID,
                                Constants.FRONT_LEFT_ENCODER_ID,
                                Constants.FRONT_LEFT_ANGLE_OFFSET);

                this.frontRight = new SwerveModule(
                                Constants.FRONT_RIGHT_DRIVE_MOTOR_ID,
                                Constants.FRONT_RIGHT_TURN_MOTOR_ID,
                                Constants.FRONT_RIGHT_ENCODER_ID,
                                Constants.FRONT_RIGHT_ANGLE_OFFSET);

                this.backLeft = new SwerveModule(
                                Constants.BACK_LEFT_DRIVE_MOTOR_ID,
                                Constants.BACK_LEFT_TURN_MOTOR_ID,
                                Constants.BACK_LEFT_ENCODER_ID,
                                Constants.BACK_LEFT_ANGLE_OFFSET);

                this.backRight = new SwerveModule(
                                Constants.BACK_RIGHT_DRIVE_MOTOR_ID,
                                Constants.BACK_RIGHT_TURN_MOTOR_ID,
                                Constants.BACK_RIGHT_ENCODER_ID,
                                Constants.BACK_RIGHT_ANGLE_OFFSET);

                this.modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };
        }

        public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
                this.speeds = speeds;
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

                for (int i = 0; i < 4; i++) {
                        modules[i].setDesiredState(states[i], isOpenLoop);
                }
        }

        public Rotation2d getHeading() {
                return null;
        }

        public void updateSmartDash() {
                for (SwerveModule module : modules) {
                        module.updateSmartDash();
                }
        }

}
