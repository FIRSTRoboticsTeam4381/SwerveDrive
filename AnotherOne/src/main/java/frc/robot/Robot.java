package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.PIDVariables;
import java.util.Timer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends IterativeRobot
{   
    private static  String kDefaultAuto = "Default";
    private static  String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private  SendableChooser<String> m_chooser;
    private Joystick m_DriveStick;
    private double angle;
    private double mag;
    private double Max1;
    private double y;
    private boolean turning;
    private double turnTo;
    private double povAngle = 0;
    public long starttime = 0;
    private Drivetrain drivetrain = new Drivetrain();

    public Robot() {
        m_DriveStick = new Joystick(0);
    }
    
    public void robotInit() {
        starttime = System.currentTimeMillis();      
    }
    
    public void robotPeriodic() { 
    }
    
    public void autonomousInit() {
    }
    
    public void autonomousPeriodic() {
    }
    
    public void teleopPeriodic() {
        long elapsedtime = System.currentTimeMillis() - starttime;
        SmartDashboard.putNumber("Timer", elapsedtime);
        
        if(m_DriveStick.getY() < 0.1 && m_DriveStick.getY() > -0.1){
            y = 0;
        }else{
            y = m_DriveStick.getY() * m_DriveStick.getThrottle();
        }



        drivetrain.runPID(-y, m_DriveStick.getPOV(), m_DriveStick.getRawButton(1), m_DriveStick.getRawButton(2));
    

        

        /*
        angle = Math.atan2(-y, x);
        mag = Math.sqrt(Math.abs((Math.pow(x, 2)) + Math.abs(Math.pow(-y, 2))));     
        if (angle < 0) {
            angle = Math.toDegrees(angle) + 360;
        } else {
            angle = Math.toDegrees(angle);
        }
        if(mag > 1){
            mag = 1;
        }
        if(mag < -1){
            mag = -1;
        }
        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber("magnitude", mag);
        */
    }
    
    public void testPeriodic() {
    }
}