package frc.lib.control.motors;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

public class NKSparkMax extends CANSparkMax {

    protected SparkMaxPIDController controller;
    protected double m_lastSpeed;
    protected int m_id;

    public NKSparkMax(int deviceID, boolean isBrushless) {
        super(deviceID, isBrushless? MotorType.kBrushless : MotorType.kBrushed);
        this.controller = this.getPIDController();
        this.m_id = deviceID;
    }

    @Override
    public void set(double speed) {
        if (this.m_lastSpeed != speed) {
            this.m_lastSpeed = speed;
            super.set(speed);
        }
        
    }

    /**
     * @return the onboard PID controller for the motor
     */
    public SparkMaxPIDController getController() {
        return controller;
    }
}