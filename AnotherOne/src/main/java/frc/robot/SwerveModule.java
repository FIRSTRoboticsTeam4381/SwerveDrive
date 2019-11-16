package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SwerveModule implements edu.wpi.first.wpilibj.PIDSource
{
    private Spark one;
    private Spark two;
    private PIDController pidControl;
    private double targetAngle;
    private double Kp = 0.002;
    private double Ki = 0;
    private double Kd = 0;
    private double rotation;
    private PIDOutput output;
    private double max = 0.8;
    public double y2 = 0;
    public double y1 = 0;
    public SwerveModule( int deviceID1,  int deviceID2)  {
        one = new Spark(deviceID1);
        two = new Spark(deviceID2);
        one.setPIDcoefficients();
        two.setPIDcoefficients();
        
        pidControl = new PIDController(Kp, Ki, Kd, this, this::SetRotation);
        pidControl.setInputRange(0, 360);
        pidControl.setOutputRange(-1, 1);
        pidControl.setContinuous(true);
        pidControl.setAbsoluteTolerance(2);
        pidControl.setSetpoint(0);
        pidControl.reset();
        //pidControl.enable();
        
        //one.setTurnPIDcoefficients();
        //two.setTurnPIDcoefficients();
    }
    
    public void SetRotation (double output){
        rotation = output;
      }

    public void angle(double angle){
        if(angle < 0){
            angle = 0;
        }
        pidControl.setSetpoint(angle);
    }

    public void runPID( double getY1, double getY2) {
        one.calculatePID(-getY1);
        two.calculatePID(getY2);
    }
    public void runTurnPID(){
        one.turnPID();
        two.turnPID();
    }
    public void updatePID(Spark motor, double newP, double newI, 
    double newD, double newIZ, double newFF, double newMaxOutput, double newMinOutput){
        motor.updatePID(newP, newI, newD, newIZ, newFF, newMaxOutput, newMinOutput);
    }
    public void updateTurnPID(Spark motor, double newP, double newI, 
    double newD, double newIZ, double newFF, double newMaxOutput, double newMinOutput, double rotations){
        motor.updateTurnPID(newP, newI, newD, newIZ, newFF, newMaxOutput, newMinOutput, rotations);
    }
    public Spark getMotor(int number){
        if(number == 1){
            return one;
        }else{
            return two;
        }
    }

    public double getAngle(Spark one, Spark two){
        double diff = one.ticks() + two.ticks();
        diff = (diff % 10) * 36;

        if(diff < 0){
            diff = diff + 360;
        }

        return diff;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return getAngle(one, two);
    }
    
    public void getYs(double mainY, double targetAngle){

        if(targetAngle != -1){
            y2 = (mainY) + rotation;
            y1 = mainY;
        }else{
            y1 = mainY;
            y2 = mainY;
        }

    }


    }