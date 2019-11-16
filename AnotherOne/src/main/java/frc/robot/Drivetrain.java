package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain
{
    private PIDVariables V;
    private SwerveModule FR;
    private SwerveModule FL;
    private SwerveModule BR;
    private SwerveModule BL;
    private double y;
    private double angle;
    

    public Drivetrain(){
        V = new PIDVariables();

        FR = new SwerveModule(50, 48);
        FL = new SwerveModule(57, 52);
        BR = new SwerveModule(60, 51);
        BL = new SwerveModule(62, 55);

        FR.updatePID(FR.getMotor(2), V.FR48("p"), V.FR48("i"), V.FR48("d"), 
        V.FR48("iz"), V.FR48("ff"), V.FR48("max"), V.FR48("min"));

        FR.updatePID(FR.getMotor(1), V.FR50("p"), V.FR50("i"), V.FR50("d"), 
        V.FR50("iz"), V.FR50("ff"), V.FR50("max"), V.FR50("min"));

        FL.updatePID(FL.getMotor(2), V.FL52("p"), V.FL52("i"), V.FL52("d"), 
        V.FL52("iz"), V.FL52("ff"), V.FL52("max"), V.FL52("min"));

        FL.updatePID(FL.getMotor(1), V.FL57("p"), V.FL57("i"), V.FL57("d"), 
        V.FL57("iz"), V.FL57("ff"), V.FL57("max"), V.FL57("min"));

        BR.updatePID(BR.getMotor(2), V.BR51("p"), V.BR51("i"), V.BR51("d"), 
        V.BR51("iz"), V.BR51("ff"), V.BR51("max"), V.BR51("min"));

        BR.updatePID(BR.getMotor(1), V.BR60("p"), V.BR60("i"), V.BR60("d"), 
        V.BR60("iz"), V.BR60("ff"), V.BR60("max"), V.BR60("min"));

        BL.updatePID(BL.getMotor(2), V.BL53("p"), V.BL53("i"), V.BL53("d"), 
        V.BL53("iz"), V.BL53("ff"), V.BL53("max"), V.BL53("min"));

        BL.updatePID(BL.getMotor(1), V.BL62("p"), V.BL62("i"), V.BL62("d"), 
        V.BL62("iz"), V.BL62("ff"), V.BL62("max"), V.BL62("min"));
    }
    
    public void runPID(double inY, double inAngle, boolean stop, boolean rst) {
        y = inY;
        //if using POV angle = inAngle
        //if using Z axis angle = inAngle * 360
        angle = inAngle;

        FR.getYs(y, angle);
        FL.getYs(y, angle);
        BR.getYs(y, angle);
        BL.getYs(y, angle);

        if(stop){
            FR.runPID(0, 0);
            FL.runPID(0, 0);
            BR.runPID(0, 0);
            BL.runPID(0, 0);
        }else{ 
            FR.runPID(FR.y1, FR.y2);
            FL.runPID(FL.y1, FL.y2);
            BR.runPID(BR.y1, BR.y2);
            BL.runPID(BL.y1, BL.y2);
        }
        
    
        SmartDashboard.putNumber("inangle", inAngle);
        FR.angle(inAngle);
        FL.angle(inAngle);
        BR.angle(inAngle);
        BL.angle(inAngle);

        if(rst){
            reset(FR);
            reset(FL);
            reset(BR);
            reset(BL);
        }


        SmartDashboard.putNumber("BL vel1", BL.getMotor(1).velo());
        SmartDashboard.putNumber("BL vel2", BL.getMotor(2).velo());
        SmartDashboard.putNumber("FR difference",  FR.getMotor(2).velo() + FR.getMotor(1).velo());
       
        SmartDashboard.putNumber("FR getAngle", FR.getAngle(FR.getMotor(2), FR.getMotor(1)));
      
        SmartDashboard.putNumber("FL difference",  FL.getMotor(2).velo() + FL.getMotor(1).velo());
        SmartDashboard.putNumber("FL getAngle", FL.getAngle(FL.getMotor(2), FL.getMotor(1)));
        SmartDashboard.putNumber("BR difference",  BR.getMotor(2).velo() + BR.getMotor(1).velo());
        SmartDashboard.putNumber("BR getAngle", BR.getAngle(BR.getMotor(2), BR.getMotor(1)));
        SmartDashboard.putNumber("BL difference",  BL.getMotor(2).velo() + BL.getMotor(1).velo());
        SmartDashboard.putNumber("BL getAngle", BL.getAngle(BL.getMotor(2), BL.getMotor(1)));

    }
   
    

    public void reset(SwerveModule mod){
        mod.getMotor(1).reset();
        mod.getMotor(2).reset();
    }
    /*
    public double within(double active, double target){
        if(((360-target)+active) < target - active){
           return (target - active)*0.002;
        }else if(((360-target)+active) > target - active){
            return -((target - active)*0.002);
         } else {
             return 0;
         }
    }
   */
    
}