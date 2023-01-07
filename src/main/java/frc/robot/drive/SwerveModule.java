package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.motors.NKTalonFX;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
    private NKTalonFX drive;
    private NKTalonFX turn;
    private CANCoder turnEncoder;
    private Rotation2d angleOffset = new Rotation2d();
    private SimpleMotorFeedforward feedforward;
    private double lastAngleDelta;

    private int id;

    private double desiredAngle = 0;

    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, Rotation2d angleOffset) {
        id = (driveMotorID / 10) - 1;
        initEncoder(encoderID);
        initDriveMotor(driveMotorID);
        initTurnMotor(turnMotorID);
        this.feedforward = new SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA);
        this.angleOffset = angleOffset;
    }

    public SwerveModuleState getCurrentState() {
        double velocity = getVelocityMPS();
        Rotation2d angle = getAngleRotation2d();
        return new SwerveModuleState(velocity, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SmartDashboard.putString("State: ", "not velocity");

        double angleDelta = getCurrentState().angle.minus(desiredState.angle).getDegrees();

        if (Math.abs(angleDelta) > 90) {
            angleDelta -= 180 * Math.signum(angleDelta);
            desiredState.speedMetersPerSecond *= -1;
        }

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.MAX_SPEED;
            drive.set(ControlMode.PercentOutput, percentOutput);
        } else {
            // SmartDashboard.putString("State: ", "velocity");
            // double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.WHEEL_CIRCUMFERENCE,
            //         Constants.DRIVE_GEAR_RATIO);
            // SmartDashboard.putNumber("Target velocity", desiredState.speedMetersPerSecond);

            // double l = feedforward.calculate(desiredState.speedMetersPerSecond) / 12;
            // SmartDashboard.putNumber("L: ", l);
            // drive.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
            //         l); // this is important

            drive.setVoltage(feedforward.calculate(desiredState.speedMetersPerSecond) + Constants.DRIVE_KP * (desiredState.speedMetersPerSecond - getVelocityMPS()));
        }

        if (Math.abs(desiredState.speedMetersPerSecond) > (Constants.MAX_SPEED * 0.02)) {
            turn.set(ControlMode.Position, turn.getSelectedSensorPosition() - angleDelta);
        }

        // lastAngleDelta = angleDelta;
    }

    /**
     * 
     * @param targetAngle -180 to 180 degrees as per WPILib
     * @param unboundedAngle unbounded angle given by
     * @return
     */
    public double calculateAngleDelta(double targetAngle, double unboundedAngle) {
        return boundPlusOrMinus180(targetAngle - boundPlusOrMinus180(unboundedAngle));
    }

    private double boundPlusOrMinus180(double unboundedAngle) {
        double remainder  = unboundedAngle % 360;
        double bounded0to360 = (remainder >= 0)? remainder : 360 + remainder;
        return bounded0to360 - 180;
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
        turn.setSensorPhase(true);
        turn.setNeutralMode(Constants.TURN_MOTOR_NEUTRAL);
        turn.configRemoteFeedbackFilter(turnEncoder, 0);
        turn.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);
        // double absPos = turnEncoder.getAbsolutePosition() - angleOffset.getDegrees();
        // turn.setSelectedSensorPosition(Conversions.degreesToFalcon(absPos, Constants.TURN_GEAR_RATIO));
    }

    private void initEncoder(int encoderID) {
        turnEncoder = new CANCoder(encoderID);

        turnEncoder.configFactoryDefault();
        turnEncoder.configAllSettings(Constants.ENCODER_CONFIGURATION);
    }

    public void updateSmartDash() {
        double dA = SmartDashboard.getNumber(id + " desired angle", desiredAngle);

        if (desiredAngle != dA) {
            desiredAngle = dA;
        }

        SmartDashboard.putNumber(id + " Module Encoder Raw Position", turnEncoder.getPosition());
        SmartDashboard.putNumber(id + " Encoder Raw Absolute Position", turnEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(id + " Motor Selected Sensor Position", turn.getSelectedSensorPosition());
        SmartDashboard.putNumber(id + " Module Angle", getAngleRotation2d().getDegrees());
        double[] positions = { turnEncoder.getPosition(), turn.getSelectedSensorPosition() };
        SmartDashboard.putNumberArray("Positions (should be equal)", positions);

        SmartDashboard.putNumber(id + " desired angle", desiredAngle);
        SmartDashboard.putNumber(id + " angle delta", getCurrentState().angle.minus(Rotation2d.fromDegrees(desiredAngle)).getDegrees());
    }

    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(boundPlusOrMinus180(turnEncoder.getPosition() - angleOffset.getDegrees()));
    }

    public double getVelocityMPS() {
        return Conversions.falconToMPS(drive.getSelectedSensorVelocity(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
    }

    public void testDriveSpinny(double output) {
        drive.set(output);
    }

    public void testTurnSpinny(double output) {
        turn.set(output);
    }

    public void testStopSpinny() {
        drive.set(0);
        turn.set(0);
    }

    public void setDriveVoltage(double voltage) {
        drive.setVoltage(voltage);
    }

    private static final class Constants {

        public static final ModuleConfiguration MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L2;

        private static final double MAX_SPEED = frc.robot.Constants.MAX_TRANSLATIONAL_VELOCITY;
        private static final double WHEEL_DIAMETER_METERS = MODULE_CONFIGURATION.getWheelDiameter();
        private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
        // ratio is motor rot / wheel rot
        private static final double DRIVE_GEAR_RATIO = 1 / MODULE_CONFIGURATION.getDriveReduction();
        private static final double TURN_GEAR_RATIO = 1 / MODULE_CONFIGURATION.getSteerReduction();

        private static final boolean DRIVE_MOTOR_INVERTED = MODULE_CONFIGURATION.isDriveInverted();
        private static final NeutralMode DRIVE_MOTOR_NEUTRAL = NeutralMode.Coast;
        private static final boolean TURN_MOTOR_INVERTED = true;
        private static final NeutralMode TURN_MOTOR_NEUTRAL = NeutralMode.Coast; // set back to brake to be amazing
                                                                                 // later
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
                this.voltageCompSaturation = 12;
                this.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;

            }
        };
        private static final TalonFXConfiguration TURN_MOTOR_CONFIGURATION = new TalonFXConfiguration() {
            {
                this.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
                this.primaryPID.selectedFeedbackCoefficient = 0.087890625;
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
                this.voltageCompSaturation = 12;
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

        private static final double DRIVE_KS = 0.519;
        private static final double DRIVE_KV = 2.306; 
        private static final double DRIVE_KA = 0.0;

        private static final double DRIVE_KP = 1.5;
        private static final double DRIVE_KI = 0.0;
        private static final double DRIVE_KD = 0.0;
        private static final double DRIVE_KF = 0; // 0.25 / 0.54 * 0.1;

        private static final double TURN_KP = 2;
        private static final double TURN_KI = 0;
        private static final double TURN_KD = 0;
        private static final double TURN_KF = 0.0;
    }

    public static final class Conversions {
        public static double falconToDegrees(double counts, double gearRatio) {
            // ratio = motor/wheel
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
}
