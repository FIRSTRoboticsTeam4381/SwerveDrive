package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    private double y;
    private SwerveModule FR;
    
    public Robot() {
        //m_chooser = (SendableChooser<String>)new SendableChooser();
        m_DriveStick = new Joystick(0);
    }
    
    public void robotInit() {
        //m_chooser.setDefaultOption("Default Auto", (Object)"Default");
        //m_chooser.addOption("My Auto", (Object)"My Auto");
        //SmartDashboard.putData("Auto choices", (Sendable)m_chooser);
        FR = new SwerveModule(48, 50);
    }
    
    public void robotPeriodic() {
    }
    
    public void autonomousInit() {
        //m_autoSelected = (String)m_chooser.getSelected();
        //System.out.println(invokedynamic(makeConcatWithConstants:(Ljava/lang/String;)Ljava/lang/String;, m_autoSelected));
    }
    
    public void autonomousPeriodic() {
         //String autoSelected = m_autoSelected;
       // switch (autoSelected) {
        //}
    }
    
    public void teleopPeriodic() {
        if (m_DriveStick.getY() < 0.15 && m_DriveStick.getY() > -0.15) {
            y = 0.0;
        }
        else {
            y = m_DriveStick.getY();
        }
        SmartDashboard.putNumber("Joystick", y);
        FR.runPID(y);
    }
    
    public void testPeriodic() {
    }
}