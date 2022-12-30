package frc.robot.drive;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.control.motors.NKTalonFX;

public class SwerveModule {
    private NKTalonFX drive;
    private NKTalonFX turn;
    private CANCoder turnEncoder;
    private double angleOffset;
    private SimpleMotorFeedforward feedforward;
    private double lastAngle;

    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, Rotation2d angleOffset) {
        initEncoder(encoderID);
        initDriveMotor(driveMotorID);
        initTurnMotor(turnMotorID);
        this.feedforward = new SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA);
        this.angleOffset = angleOffset.getDegrees();
        new Rotation2d(0);
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(drive.getSelectedSensorVelocity(), Constants.WHEEL_CIRCUMFERENCE,
                Constants.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d
                .fromDegrees(Conversions.falconToDegrees(turn.getSelectedSensorPosition(), Constants.TURN_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
        state = optimizeAngle(state, getState().angle);

        if (isOpenLoop) {
            double percentOutput = state.speedMetersPerSecond / Constants.MAX_SPEED;
            drive.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(state.speedMetersPerSecond, Constants.WHEEL_CIRCUMFERENCE,
                    Constants.DRIVE_GEAR_RATIO);
            drive.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(state.speedMetersPerSecond));
        }

        double angle = (Math.abs(state.speedMetersPerSecond) <= (Constants.MAX_SPEED * 0.01)) ? lastAngle
                : state.angle.getDegrees();
        turn.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.TURN_GEAR_RATIO));
        lastAngle = angle;
    }

    private void initDriveMotor(int driveMotorID) {
        drive = new NKTalonFX(driveMotorID);

        drive.configFactoryDefault();
        drive.configAllSettings(Constants.DRIVE_MOTOR_CONFIGURATION);
        drive.setInverted(Constants.DRIVE_MOTOR_INVERTED);
        drive.setNeutralMode(Constants.DRIVE_MOTOR_NEUTRAL);
        drive.setSelectedSensorPosition(0);

    }

    private void initTurnMotor(int turnMotorID) {
        turn = new NKTalonFX(turnMotorID);

        turn.configFactoryDefault();
        turn.configAllSettings(Constants.TURN_MOTOR_CONFIGURATION);
        turn.setInverted(Constants.TURN_MOTOR_INVERTED);
        turn.setNeutralMode(Constants.TURN_MOTOR_NEUTRAL);
        turn.setSelectedSensorPosition(Conversions.degreesToFalcon(getAngleDegrees(), Constants.TURN_GEAR_RATIO));
    }

    private void initEncoder(int encoderID) {
        turnEncoder = new CANCoder(encoderID);

        turnEncoder.configFactoryDefault();
        turnEncoder.configAllSettings(Constants.ENCODER_CONFIGURATION);
    }

    public Rotation2d getEncoder() {
        return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());
    }

    public double getAngleDegrees() {
        return getEncoder().getDegrees() - angleOffset;
    }

    private static final class Constants {

        public static final ModuleConfiguration MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L2;

        private static final double MAX_SPEED = frc.robot.Constants.MAX_TRANSLATIONAL_VELOCITY;
        private static final double WHEEL_DIAMETER_METERS = MODULE_CONFIGURATION.getWheelDiameter();
        private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
        private static final double DRIVE_GEAR_RATIO = MODULE_CONFIGURATION.getDriveReduction();
        private static final double TURN_GEAR_RATIO = MODULE_CONFIGURATION.getSteerReduction();
        
        private static final boolean DRIVE_MOTOR_INVERTED = MODULE_CONFIGURATION.isDriveInverted();
        private static final NeutralMode DRIVE_MOTOR_NEUTRAL = NeutralMode.Coast;
        private static final boolean TURN_MOTOR_INVERTED = MODULE_CONFIGURATION.isSteerInverted();
        private static final NeutralMode TURN_MOTOR_NEUTRAL = NeutralMode.Brake;
        private static final boolean ENCODER_INVERTED = false;

        private static final TalonFXConfiguration DRIVE_MOTOR_CONFIGURATION = new TalonFXConfiguration() {
            {
                this.slot0.kP = DRIVE_KP;
                this.slot0.kI = DRIVE_KI;
                this.slot0.kD = DRIVE_KD;
                this.slot0.kF = DRIVE_KF;
                this.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                        DRIVE_ENABLE_CURRENT_LIMIT,
                        DRIVE_CONTINUOUS_CURRENT_LIMIT,
                        DRIVE_PEAK_CURRENT_LIMIT,
                        DRIVE_PEAK_CURRENT_DURATION);

                this.initializationStrategy = SensorInitializationStrategy.BootToZero;
                this.openloopRamp = OPEN_LOOP_RAMP;
                this.closedloopRamp = CLOSED_LOOP_RAMP;

            }
        }; 
        private static final TalonFXConfiguration TURN_MOTOR_CONFIGURATION = new TalonFXConfiguration() {
            {
                this.slot0.kP = TURN_KP;
                this.slot0.kI = TURN_KI;
                this.slot0.kD = TURN_KD;
                this.slot0.kF = TURN_KF;
                this.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                        TURN_ENABLE_CURRENT_LIMIT,
                        TURN_CONTINUOUS_CURRENT_LIMIT,
                        TURN_PEAK_CURRENT_LIMIT,
                        TURN_PEAK_CURRENT_DURATION);

                this.initializationStrategy = SensorInitializationStrategy.BootToZero;
            }
        };
        private static final CANCoderConfiguration ENCODER_CONFIGURATION = new CANCoderConfiguration() {
            {
                this.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                this.sensorDirection = ENCODER_INVERTED;
                this.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                this.sensorTimeBase = SensorTimeBase.PerSecond;
            }
        };

        private static final double OPEN_LOOP_RAMP = 0.25;
        private static final double CLOSED_LOOP_RAMP = 0.0;

        private static final boolean TURN_ENABLE_CURRENT_LIMIT = true;
        private static final int TURN_CONTINUOUS_CURRENT_LIMIT = 25;
        private static final int TURN_PEAK_CURRENT_LIMIT = 40;
        private static final double TURN_PEAK_CURRENT_DURATION = 0.1;

        private static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
        private static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        private static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        private static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;

        private static final double DRIVE_KS = (0.667 / 12);
        private static final double DRIVE_KV = (2.44 / 12);
        private static final double DRIVE_KA = (0.27 / 12);

        private static final double DRIVE_KP = 0.10;
        private static final double DRIVE_KI = 0.0;
        private static final double DRIVE_KD = 0.0;
        private static final double DRIVE_KF = 0.0;

        private static final double TURN_KP = 0.6;
        private static final double TURN_KI = 0.0;
        private static final double TURN_KD = 12.0;
        private static final double TURN_KF = 0.0;
    }

    public static final class Conversions {
        public static double falconToDegrees(double counts, double gearRatio) {
            return counts * (360.0 / (gearRatio * 2048.0));
        }

        public static double degreesToFalcon(double degrees, double gearRatio) {
            double ticks = degrees / (360.0 / (gearRatio * 2048.0));
            return ticks;
        }

        public static double falconToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * (600.0 / 2048.0);
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        public static double RPMToFalcon(double RPM, double gearRatio) {
            double motorRPM = RPM * gearRatio;
            double sensorCounts = motorRPM * (2048.0 / 600.0);
            return sensorCounts;
        }

        public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
            double wheelRPM = falconToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / 60;
            return wheelMPS;
        }

        public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
            double wheelRPM = ((velocity * 60) / circumference);
            double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
            return wheelVelocity;
        }
    }

    private static final SwerveModuleState optimizeAngle(SwerveModuleState state, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), state.angle.getDegrees());
        double targetSpeed = state.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
}
