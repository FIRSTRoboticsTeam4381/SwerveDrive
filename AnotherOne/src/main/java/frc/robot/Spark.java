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
    private double kTurnP;
    private double kTurnI;
    private double kTurnD;
    private double kTurnIz;
    private double kTurnFF;
    private double kTurnMaxOutput;
    private double kTurnMinOutput;
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
    
    public void speed(double setSpeed){
        m_motor.set(setSpeed);
    }

    public double conversion(){
        return m_encoder.getPositionConversionFactor();

    }

    public double velo(){
        return m_encoder.getVelocity();
    }
    public double ticks(){
        return m_encoder.getPosition();
    }

    public void reset(){
        m_encoder.setPosition(0);
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
        m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10);
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        SmartDashboard.putNumber("P Gain" + deviceid, kP);
        SmartDashboard.putNumber("P Gain" + deviceid, kI);
        SmartDashboard.putNumber("D Gain" + deviceid, kD);
        SmartDashboard.putNumber("I Zone" + deviceid, kIz);
        SmartDashboard.putNumber("Feed Forward" + deviceid, kFF);
        SmartDashboard.putNumber("Max Output" + deviceid, kMaxOutput);
        SmartDashboard.putNumber("Min Output" + deviceid, kMinOutput);
    }
    public void setTurnPIDcoefficients() {
        kTurnP = 0.1; 
        kTurnI = 1e-4;
        kTurnD = 1; 
        kTurnIz = 0; 
        kTurnFF = 0; 
        kTurnMaxOutput = 1; 
        kTurnMinOutput = -1;


        m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 1);
        m_pidController.setP(kTurnP);
        m_pidController.setI(kTurnI);
        m_pidController.setD(kTurnD);
        m_pidController.setIZone(kTurnIz);
        m_pidController.setFF(kTurnFF);
        m_pidController.setOutputRange(kTurnMinOutput, kTurnMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Turn P Gain" + deviceid, kTurnP);
        SmartDashboard.putNumber("Turn I Gain" + deviceid, kTurnI);
        SmartDashboard.putNumber("Turn D Gain" + deviceid, kTurnD);
        SmartDashboard.putNumber("Turn I Zone" + deviceid, kTurnIz);
        SmartDashboard.putNumber("Turn Feed Forward" + deviceid, kTurnFF);
        SmartDashboard.putNumber("Turn Max Output" + deviceid, kTurnMaxOutput);
        SmartDashboard.putNumber("Turn Min Output" + deviceid, kTurnMinOutput);
        SmartDashboard.putNumber("Turn Set Rotations" + deviceid, 0);
    }
    
    public void setInverted( boolean inverted) {
        m_motor.setInverted(inverted);
    }
    
    public void updatePID(double newP, double newI, double newD, double newIZ,
     double newFF, double newMaxOutput, double newMinOutput){
        SmartDashboard.putNumber("P Gain" + deviceid, newP);
        SmartDashboard.putNumber("I Gain" + deviceid, newI);
        SmartDashboard.putNumber("D Gain" + deviceid, newD);
        SmartDashboard.putNumber("I Zone" + deviceid, newIZ);
        SmartDashboard.putNumber("Feed Forward" + deviceid, newFF);
        SmartDashboard.putNumber("Max Output" + deviceid, newMaxOutput);
        SmartDashboard.putNumber("Min Output" + deviceid, newMinOutput);
        
    }
    public void updateTurnPID(double newP, double newI, double newD, double newIZ,
     double newFF, double newMaxOutput, double newMinOutput, double rotations){
        SmartDashboard.putNumber("Turn P Gain" + deviceid, kTurnP);
        SmartDashboard.putNumber("Turn I Gain" + deviceid, kTurnI);
        SmartDashboard.putNumber("Turn D Gain" + deviceid, kTurnD);
        SmartDashboard.putNumber("Turn I Zone" + deviceid, kTurnIz);
        SmartDashboard.putNumber("Turn Feed Forward" + deviceid, kTurnFF);
        SmartDashboard.putNumber("Turn Max Output" + deviceid, kTurnMaxOutput);
        SmartDashboard.putNumber("Turn Min Output" + deviceid, kTurnMinOutput);
        SmartDashboard.putNumber("Turn Set Rotations" + deviceid, rotations);
    }

    public void calculatePID( double getY) {
        double p = SmartDashboard.getNumber("P Gain" + deviceid, 0);
        double i = SmartDashboard.getNumber("I Gain" + deviceid, 0);
        double d = SmartDashboard.getNumber("D Gain" + deviceid, 0);
        double iz = SmartDashboard.getNumber("I Zone" + deviceid, 0);
        double ff = SmartDashboard.getNumber("Feed Forward" + deviceid, 0);
        double max = SmartDashboard.getNumber("Max Output" + deviceid, 0);
        double min = SmartDashboard.getNumber("Min Output" + deviceid, 0);
    
        


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

        if(getY == 0){
           m_pidController.setIAccum(0);
        }       

        double setPoint = getY * maxRPM;

        m_pidController.setReference(setPoint, ControlType.kVelocity);
        SmartDashboard.putNumber("SetPoint" + deviceid, setPoint);
        SmartDashboard.putNumber("ProcessVariable" + deviceid, m_encoder.getVelocity());
    }

    public void turnPID(){
        double p = SmartDashboard.getNumber("Turn P Gain" + deviceid, 0);
        double i = SmartDashboard.getNumber("Turn I Gain" + deviceid, 0);
        double d = SmartDashboard.getNumber("Turn D Gain" + deviceid, 0);
        double iz = SmartDashboard.getNumber("Turn I Zone" + deviceid, 0);
        double ff = SmartDashboard.getNumber("Turn Feed Forward" + deviceid, 0);
        double max = SmartDashboard.getNumber("Turn Max Output" + deviceid, 0);
        double min = SmartDashboard.getNumber("Turn Min Output" + deviceid, 0);
        double rotations = SmartDashboard.getNumber("Turn Set Rotations" + deviceid, 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kTurnP)) { m_pidController.setP(p); kTurnP = p; }
        if((i != kTurnI)) { m_pidController.setI(i); kTurnI = i; }
        if((d != kTurnD)) { m_pidController.setD(d); kTurnD = d; }
        if((iz != kTurnIz)) { m_pidController.setIZone(iz); kTurnIz = iz; }
        if((ff != kTurnFF)) { m_pidController.setFF(ff); kTurnFF = ff; }
        if((max != kTurnMaxOutput) || (min != kTurnMinOutput)) { 
          m_pidController.setOutputRange(min, max); 
          kTurnMinOutput = min; kTurnMaxOutput = max; 
        }
    
        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.ControlType.kDutyCycle
         *  com.revrobotics.ControlType.kPosition
         *  com.revrobotics.ControlType.kVelocity
         *  com.revrobotics.ControlType.kVoltage
         */
        m_pidController.setReference(rotations, ControlType.kVelocity);
        
        SmartDashboard.putNumber("SetPoint" + deviceid, rotations);
        SmartDashboard.putNumber("ProcessVariable" + deviceid, m_encoder.getPosition());


    }
}