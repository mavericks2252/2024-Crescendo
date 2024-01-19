// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  double tx = 0;
  double ty = 0;
  double ta = 0;
  boolean tv = false;
  public VisionSubsystem() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    tx = LimelightHelpers.getTX("limelight");
    SmartDashboard.putNumber("tx", tx);
    ty = LimelightHelpers.getTY("limelight");
    SmartDashboard.putNumber("ty", ty);
    ta = LimelightHelpers.getTA("limelight");
    SmartDashboard.putNumber("ta", ta);
    tv = LimelightHelpers.getTV("limelight");
    SmartDashboard.putBoolean("tv", tv);
    LimelightHelpers.setLEDMode_ForceOff("limelight");

    //if limelight has a target, it prints these numbers
    if(tv){
      SmartDashboard.putNumber("Bot X Position", getBotXPosition());
      SmartDashboard.putNumber("Bot Y Position", getBotYPosition());
      SmartDashboard.putNumber("Bot Rotation", getBotRotation());
      SmartDashboard.putNumber("Distance to Speaker", getSpeakerDistance());
      SmartDashboard.putNumber("bot target angle", getSpeakerAngle());
    }
    
    
  }
//Bot X Position In Inches
  public double getBotXPosition() {

double xMeters = LimelightHelpers.getBotPose2d_wpiBlue("limelight").getX();
return xMeters * 3.2808 * 12;

  }
//Bot Y Position In Inches
  public double getBotYPosition() {
double yMeters = LimelightHelpers.getBotPose2d_wpiBlue("limelight").getY();
return yMeters * 3.2808 * 12;


  }
//Bot Rotation In Degrees
  public double getBotRotation() {

    return LimelightHelpers.getBotPose2d_wpiBlue("limelight").getRotation().getDegrees();

  }

  public double getSpeakerDistance() {

    double botX = getBotXPosition();
    double botY = getBotYPosition();
    double oppositetSide = botY - FieldConstants.kBlueSpeakerYPos;
    return Math.sqrt((botX*botX)+(oppositetSide*oppositetSide));

  }

  public double getSpeakerAngle() {


    double botX = getBotXPosition();
    double boty = getBotYPosition();
    double oppositetSide = boty - FieldConstants.kBlueSpeakerYPos;
    double adjacentSide = botX - FieldConstants.kBlueSpeakerXPos;
    double turnangle =  Math.toDegrees(Math.tanh(oppositetSide/adjacentSide));
    SmartDashboard.putNumber("turn angle", turnangle);
    return 180 + turnangle;

  }

 
}
