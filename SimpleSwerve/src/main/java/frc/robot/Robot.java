/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.cameraserver.CameraServer;
import com.revrobotics.CANAnalog;
public class Robot extends IterativeRobot {
 
  private SwerveMod FR;
  private SwerveMod FL;
  private SwerveMod BR;
  private SwerveMod BL;

  private double FWD;
  private double RCW;
  private double STR;

  private double A;
  private double B;
  private double C;
  private double D;

  private double wsFR;
  private double wsFL;
  private double wsBL;
  private double wsBR;

  private double waFR;
  private double waFL;
  private double waBL;
  private double waBR;

  private double max;

  private double L;
  private double W;
  private double Diagonal;

  private CANAnalog FRAbsolute;
  private CANAnalog FLAbsolute;
  private CANAnalog BRAbsolute;
  private CANAnalog BLAbsolute;

  private double FRMaxDraw = 0;
  private double FLMaxDraw = 0;
  private double BRMaxDraw = 0;
  private double BLMaxDraw = 0;

  private double temp;
  public AHRS ahrs;
  private double angle;

  private double ScaleSpeed;

  private Joystick driveStick = new Joystick(0);

  @Override
  public void robotInit() {
    FR = new SwerveMod(2, 59);
    FL = new SwerveMod(49, 61);
    BR = new SwerveMod(3, 1);
    BL = new SwerveMod(56, 4);

    FRAbsolute = new CANAnalog(FR.getTurn(), CANAnalog.AnalogMode.kAbsolute);
    FLAbsolute = new CANAnalog(FL.getTurn(), CANAnalog.AnalogMode.kAbsolute);
    BRAbsolute = new CANAnalog(BR.getTurn(), CANAnalog.AnalogMode.kAbsolute);
    BLAbsolute = new CANAnalog(BL.getTurn(), CANAnalog.AnalogMode.kAbsolute);

    ahrs = new AHRS(SPI.Port.kMXP);

    ahrs.reset();

    CameraServer.getInstance().startAutomaticCapture();
    
  }

  
  @Override
  public void robotPeriodic() {
  }


  @Override
  public void autonomousInit() {
  }

  
  @Override
  public void autonomousPeriodic() {
  }

 
  public void teleopPeriodic() {

    if(driveStick.getPOV() == 0){
      FWD = 0.1;
    }else 
    if(driveStick.getPOV() == 180){
      FWD = -0.1;
    }else
    if(driveStick.getY() > -0.2 && driveStick.getY() < 0.2){
      FWD = 0;
    }else{
      FWD = -driveStick.getY();
    }
    if(driveStick.getPOV() == 90){
      STR = 0.1;
    }else
    if(driveStick.getPOV() == 270){
      STR = -0.1;
    }else
    if(driveStick.getX() > -0.2 && driveStick.getX() < 0.2){
      STR = 0;
    }else{
      STR = driveStick.getX();
    }
    if(driveStick.getZ() > -0.2 && driveStick.getZ() < 0.2){
      RCW = 0;
    }else{
      RCW = driveStick.getZ();
    }

    ScaleSpeed = (-driveStick.getThrottle() + 1)/2;
    
    SmartDashboard.putNumber("Angle", ahrs.getAngle());

    if (driveStick.getRawButton(2)){
    angle = 0;
    }else{
      angle = ahrs.getAngle();
    }

    if(driveStick.getRawButton(3)){
      ahrs.reset();
    }
    
    double gyro = (angle * Math.PI) / 180;
		double temp = FWD * Math.cos(gyro) + STR * Math.sin(gyro);
		STR = -FWD * Math.sin(gyro) + STR * Math.cos(gyro);
		FWD = temp;
    

    L = 24;
    W = 24;
    Diagonal = Math.sqrt(((Math.pow(L, 2)) + (Math.pow(W, 2))));


    
    A = STR - RCW * (L/Diagonal);
    B = STR + RCW * (L/Diagonal);
    C = FWD - RCW * (W/Diagonal);
    D = FWD + RCW * (W/Diagonal);

    wsFR = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    wsFL = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    wsBL = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
    wsBR = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

    waFR = Math.atan2(B, C) * 180/Math.PI;
    waFL = Math.atan2(B, D) * 180/Math.PI;
    waBL = Math.atan2(A, D) * 180/Math.PI;
    waBR = Math.atan2(A, C) * 180/Math.PI;

    max = wsFR;

    if(wsFL>max)
      max=wsFL;
    if(wsBL>max)
      max=wsBL;
    if(wsBR>max)
      max=wsBR;

    if(max>1){
      wsFR/=max;
      wsFL/=max;
      wsBL/=max;
      wsBR/=max;
    }
    


   if(!driveStick.getRawButton(1)){
    wsFR*=ScaleSpeed;
    wsFL*=ScaleSpeed;
    wsBR*=ScaleSpeed;
    wsBL*=ScaleSpeed;
   }

   if(driveStick.getRawButton(5)){
    FR.setPowers(0, 315);
    FL.setPowers(0, 45);
    BR.setPowers(0, 45);
    BL.setPowers(0, 315);
   }else if(driveStick.getRawButton(6)){
    FR.setPowers(0, 45);
    FL.setPowers(0, 315);
    BR.setPowers(0, 315);
    BL.setPowers(0, 45);
   }else{
    FR.setPowers(-wsFR, 360 - waFR);
    FL.setPowers(wsFL, 360 - waFL);
    BR.setPowers(-wsBR, 360 - waBR);
    BL.setPowers(wsBL, 360 - waBL);
   }
  
  if(FR.getDrive().getOutputCurrent() > FRMaxDraw){
    FRMaxDraw = FR.getDrive().getOutputCurrent();}
  if(FL.getDrive().getOutputCurrent() > FLMaxDraw){
    FLMaxDraw = FL.getDrive().getOutputCurrent();}
  if(BR.getDrive().getOutputCurrent() > BRMaxDraw){
    BRMaxDraw = BR.getDrive().getOutputCurrent();}
  if(BL.getDrive().getOutputCurrent() > BLMaxDraw){
    BLMaxDraw = BL.getDrive().getOutputCurrent();}  


   SmartDashboard.putNumber("FRAbsolute", FRAbsolute.getPosition() * 360);
   SmartDashboard.putNumber("FLAbsolute", FLAbsolute.getPosition() * 360);
   SmartDashboard.putNumber("BRAbsolute", BRAbsolute.getPosition() * 360);
   SmartDashboard.putNumber("BLAbsolute", BLAbsolute.getPosition() * 360);

   SmartDashboard.putNumber("Current Draw FR", FR.getDrive().getOutputCurrent());
   SmartDashboard.putNumber("Current Draw FL", FL.getDrive().getOutputCurrent());
   SmartDashboard.putNumber("Current Draw BR", BR.getDrive().getOutputCurrent());
   SmartDashboard.putNumber("Current Draw BL", BL.getDrive().getOutputCurrent());
  
   SmartDashboard.putNumber("Max Current Draw FR", FRMaxDraw);
   SmartDashboard.putNumber("Max Current Draw FL", FLMaxDraw);
   SmartDashboard.putNumber("Max Current Draw BR", BRMaxDraw);
   SmartDashboard.putNumber("Max Current Draw BL", BLMaxDraw);

   SmartDashboard.putNumber("SetPoint", waBL);
   
   
  
  }


  @Override
  public void testPeriodic() {
  }
}
