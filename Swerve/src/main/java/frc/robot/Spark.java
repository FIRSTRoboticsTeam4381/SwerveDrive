package frc.robot;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

public class Spark
{
    private CANSparkMax m_motor;
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    private double kP;
    private double kI;
    private double kD;
    private double kIz;
    private double kFF;
    private double kMaxOutput;
    private double kMinOutput;
    private double maxRPM;
    private double deviceid;
    
    public Spark( int deviceID) {
        m_motor = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        deviceid = deviceID;
        m_pidController = m_motor.getPIDController();
    }
    
    public void setPIDcoefficients() {
        kP = 5.0E-5;
        kI = 1.0E-6;
        kD = 0.0;
        kIz = 0.0;
        kFF = 0.0;
        kMaxOutput = 1.0;
        kMinOutput = -1.0;
        maxRPM = 5700.0;
        m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 1);
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
    }
    
    public void setInverted( boolean inverted) {
        m_motor.setInverted(inverted);
    }
    
    public void updatePID(double newP, double newI, double newD, double newIZ, double newFF){
        SmartDashboard.putNumber("P Gain", newP);
        SmartDashboard.putNumber("I Gain", newI);
        SmartDashboard.putNumber("D Gain", newD);
        SmartDashboard.putNumber("I Zone", newIZ);
        SmartDashboard.putNumber("Feed Forward", newFF);
        
    }

    public void calculatePID( double getY) {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
    
        if (p != kP) {
            m_pidController.setP(p);
            kP = p;
        }
        if (i != kI) {
            m_pidController.setI(i);
            kI = i;
        }
        if (d != kD) {
            m_pidController.setD(d);
            kD = d;
        }
        if (iz != kIz) {
            m_pidController.setIZone(iz);
            kIz = iz;
        }
        if (ff != kFF) {
            m_pidController.setFF(ff);
            kFF = ff;
        }
        if (max != kMaxOutput || min != kMinOutput) {
            m_pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
         double setPoint = getY * maxRPM;
        m_pidController.setReference(setPoint, ControlType.kVelocity);
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    }
}