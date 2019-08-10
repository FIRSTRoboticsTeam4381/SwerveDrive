package frc.robot;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.RobotBase;

public  class Main
{
    private Main() {
    }
    
    public static void main( String... args) {
        RobotBase.startRobot((Supplier)Robot::new);
    }
}