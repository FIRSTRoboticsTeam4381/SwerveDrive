package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;



import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveMod {

    
    private CANSparkMax drive;
    private CANEncoder driveEnc;
    private CANSparkMax turn;
    private CANEncoder turnEnc;
    private CANPIDController pid;
    private double angle = 0;
    private double currentAngle;
    private double drivePower = 0;
    private double turnPower = 0;
    private double kP = 0.1;
    private double kI = 1e-4;
    private double kD = 1;
    private double kIz = 0;
    private double kFF = 0;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;
    private double setPoint = 0;
    private double targetTicks = 0;

    public SwerveMod(int driveCAN, int turnCAN) {
        drive = new CANSparkMax(driveCAN, CANSparkMaxLowLevel.MotorType.kBrushless);
        turn = new CANSparkMax(turnCAN, CANSparkMaxLowLevel.MotorType.kBrushless);
        driveEnc = new CANEncoder(drive);
        turnEnc = new CANEncoder(turn);
        pid = turn.getPIDController();
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(kMinOutput, kMaxOutput);
        drive.setSmartCurrentLimit(50);
        turn.setSmartCurrentLimit(50);
        
        
    }



   

    public void setPowers(double y, double targetAngle){
       
        targetTicks = (18*targetAngle)/360;
        

      
        pid.setReference(getSetPoint(targetTicks, turnEnc.getPosition()), ControlType.kPosition);
        drivePower = y;

        drive.set(drivePower);
        
        SmartDashboard.putNumber("Set point SwerveMod", getSetPoint(targetTicks, turnEnc.getPosition()));
        SmartDashboard.putNumber("Output", turnEnc.getVelocity());
        SmartDashboard.putNumber("target ticks", targetTicks);
        SmartDashboard.putNumber("targetAngle", targetAngle);
        
        
    }
    
    public CANSparkMax getTurn(){
        return turn;
    }

    public CANEncoder getTurnEnc(){
        return turnEnc;
    }

    public CANSparkMax getDrive(){
        return drive;
    }




    private double getSetPoint(double d, double c){
        double cnormKey = 18*Math.floor(c/18);
        c = c - cnormKey;
        double dnormkey = 18*Math.floor(d/18);
        d = d - dnormkey;
        double setPoint = c;
        double c1 = d;
        double c2 = d+18;
        double c3 = d-18;
        double Dc1 = Math.abs(c-c1);
        double Dc2 = Math.abs(c-c2);
        double Dc3 = Math.abs(c-c3);

        if(Dc1 < Dc2){
            if(Dc1 < Dc3){
                setPoint = c1;
            }
        }else if(Dc2 < Dc3){
            if(Dc2 < Dc1){
                setPoint = c2;
            }
        }else{
            setPoint = c3;
        }
            
        return setPoint + cnormKey;
    }






}