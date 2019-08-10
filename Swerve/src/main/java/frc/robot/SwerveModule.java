package frc.robot;

public class SwerveModule
{
    private Spark one;
    private Spark two;
    
    public SwerveModule( int deviceID1,  int deviceID2) {
        one = new Spark(deviceID1);
        two = new Spark(deviceID2);
        one.setPIDcoefficients();
        two.setPIDcoefficients();
    }
    
    public void runPID( double getY) {
        one.calculatePID(-getY);
        two.calculatePID(getY);
    }
}