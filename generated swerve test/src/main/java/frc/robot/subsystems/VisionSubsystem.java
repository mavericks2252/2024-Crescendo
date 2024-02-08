// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  double tx = 0;
  double ty = 0;
  double ta = 0;
  boolean tv = false;
  CommandSwerveDrivetrain drivetrain;
  Field2d mField2d;
  String limelight = "limelight";
  
  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {

    
    this.drivetrain = drivetrain;
    mField2d = new Field2d();
    LimelightHelpers.setLEDMode_ForceOff(limelight);
    SmartDashboard.putData("Field", mField2d);
    mField2d.setRobotPose(new Pose2d(3, 4, new Rotation2d(Math.toRadians(45))));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    tx = LimelightHelpers.getTX(limelight);
    SmartDashboard.putNumber("tx", tx);
    ty = LimelightHelpers.getTY(limelight);
    SmartDashboard.putNumber("ty", ty);
    ta = LimelightHelpers.getTA(limelight);
    SmartDashboard.putNumber("ta", ta);
    tv = LimelightHelpers.getTV(limelight);
    SmartDashboard.putBoolean("tv", tv);
    
    SmartDashboard.putBoolean("Aliance Color", driverStationAlliance());
    SmartDashboard.putNumber("latency", getTotalLatency());
    SmartDashboard.putNumber("FPGA time", Timer.getFPGATimestamp());

    SmartDashboard.putNumber("speakerDistance", getSpeakerDistance());
    SmartDashboard.putNumber("Target Angle", getTargetAngle());

    SmartDashboard.putNumber("Amp Distance", getAmpDistance());
    
    

    //if limelight has a target, it prints these numbers
    if(tv){
      SmartDashboard.putNumber("New Target angle test", getSpeakerTargetRotation2d().getRotation().getDegrees());
      SmartDashboard.putString("bot Pose", LimelightHelpers.getBotPose2d_wpiBlue(limelight).toString());
      }
    
    

    /**addVisionRobotPose();
    Pose2d currentRobotPos = drivetrain.getState().Pose; //sets currentRobotPos to the position of the robot
    SmartDashboard.putString("Robto OdometryPose", currentRobotPos.toString());
    mField2d.setRobotPose(currentRobotPos); **/
     
     
    
  }

  public Pose2d getCurrentPose2d(){
    return drivetrain.getState().Pose; // sets the method getCurrentPose2d to the value of the current pose
  }


  // calculate rotation to speaker
  public Pose2d getSpeakerTargetRotation2d(){
    double xRobotPosMeters = getCurrentPose2d().getX(); //gets the x pose of the bot
    double yRobotPosMeters = getCurrentPose2d().getY(); //gets the y pose of the bot
    Translation2d speakerPos; //gets the position of the speaker

    if (driverStationAlliance()){
      speakerPos = FieldConstants.kRedSpeaker; //sets the location of the speaker for the red side

    }

    else {
      speakerPos = FieldConstants.kBlueSpeaker; //sets the location of the speaker for the blue side

    }
    
    //double xRobotPosMeters = LimelightHelpers.getBotPose2d_wpiBlue(limelight).getX(); //get xpose of the robot
    //double yRobotPosMeters = LimelightHelpers.getBotPose2d_wpiBlue(limelight).getY();  // get the ypose of the robot

    // return a pose 2d of robot location and target angle of speaker
    return new Pose2d(xRobotPosMeters, yRobotPosMeters, (new Rotation2d(speakerPos.getX() - xRobotPosMeters, speakerPos.getY() - yRobotPosMeters))); //finds the pose of the robot using the location and angle of the speaker
    

  }

  

  // get the robotPose from Vision based on alliance color
  public Pose2d getRobotPoseVision () {
    
    return LimelightHelpers.getBotPose2d_wpiBlue(limelight); //gets the pose of the bot using the limelight
   
  }

  // get the total latency of target from Limelight
  public double getTotalLatency (){ //returns total latency in seconds
    double tl = LimelightHelpers.getLatency_Pipeline(limelight);
    double cl = LimelightHelpers.getLatency_Capture(limelight);
    SmartDashboard.putNumber("total Latencey",  tl + cl);
    return (tl + cl)/1000; 
  }

  //add a vision measurement to the drivetrain odemetry to update it
  //if the target is present and large enough on screen
  public void addVisionRobotPose(){

    if (tv && ta >.75){
      drivetrain.addVisionMeasurement(getRobotPoseVision(), Timer.getFPGATimestamp() - getTotalLatency()); //corrects the odometry pose and takes in the latency offset
      SmartDashboard.putBoolean("vision pose added", true);
    }
    else{
      SmartDashboard.putBoolean("vision pose added", false);
    }
    

  }

  public void seedRobotPoseFromVision(){

    if(tv) {
    drivetrain.seedFieldRelative(getRobotPoseVision()); //seeds the robot when it sees a tag
    }

  }

  public boolean driverStationAlliance(){
    var alliance = DriverStation.getAlliance(); //creates a variable called alliance
    if (alliance.isPresent()) { //checks if alliance variable has been set to anything
      return alliance.get() == DriverStation.Alliance.Red; //checks if the alliance color is red
    }
    return false; //returns false if color is not red or if no color
  }

  public double getSpeakerDistance() {

    double botX = drivetrain.getState().Pose.getX();
    double botY = drivetrain.getState().Pose.getY();
    double oppositetSide = botY - FieldConstants.kBlueSpeakerYPosMeters;
    
    
    
    return Math.hypot(oppositetSide, botX);
    //return Math.sqrt((botX*botX)+(oppositetSide*oppositetSide));

  }
  public double getTargetAngle() {
    double speakerToPivot = FieldConstants.kPivotToSpeaker;
    
   
    return Math.toDegrees(Math.atan2(speakerToPivot, getSpeakerDistance()));
  }

  public double getAmpDistance() {
    double botX = drivetrain.getState().Pose.getX();
    double botY = drivetrain.getState().Pose.getY();
    double oppositetSide = botY - FieldConstants.kBlueAmpYPosMeters;

    return Math.hypot(oppositetSide, botX);
  }



















/*
  //Bot X Position In Inches
  public double getBotXPosition() {

    double xMeters = LimelightHelpers.getBotPose2d_wpiBlue(limelight).getX();
    return xMeters * 3.2808 * 12;

  }
//Bot Y Position In Inches
  public double getBotYPosition() {
    double yMeters = LimelightHelpers.getBotPose2d_wpiBlue(limelight).getY();
    return yMeters * 3.2808 * 12;


  }
//Bot Rotation In Degrees
  public double getBotRotation() {

    return LimelightHelpers.getBotPose2d_wpiBlue(limelight).getRotation().getDegrees();

  }

  public double getSpeakerDistance() {

    double botX = getBotXPosition();
    double botY = getBotYPosition();
    double oppositetSide = botY - FieldConstants.kBlueSpeakerYPos;
    return Math.hypot(botX, oppositetSide);
    //return Math.sqrt((botX*botX)+(oppositetSide*oppositetSide));

  }

  public double getSpeakerTurnAngle() {


    double botX = getBotXPosition();
    double boty = getBotYPosition();
    double oppositetSide = boty - FieldConstants.kBlueSpeakerYPos;
    double adjacentSide = botX - FieldConstants.kBlueSpeakerXPos;
    return - Math.toDegrees(Math.tanh(oppositetSide/adjacentSide));
    

  }

  public double getBotTargetAngle() {
    return Math.copySign(180, getSpeakerTurnAngle()) - getSpeakerTurnAngle();
  }

  public double getSpeakerError() {

    double error = 0;
    if(getBotRotation()>0 && getBotTargetAngle()<0){
      error = -(180 - getBotRotation())  +  (-180 - getBotTargetAngle());
    
    }
    else if(getBotRotation()<0 && getBotTargetAngle()>0){
      error = (180 - getBotTargetAngle()) + (180 - Math.abs(getBotRotation()));

    }
    else{
      error = getBotRotation() - getBotTargetAngle();
    }
    
    return error;
  }


  public double getSteeringPercentage() {

    double steeringPercentage = getSpeakerError()/50;
    //
    steeringPercentage = (tv) ? steeringPercentage : 0;
    steeringPercentage = (steeringPercentage>0.5) ? 0.5 : steeringPercentage;
    steeringPercentage = (steeringPercentage<-0.5) ? -0.5 : steeringPercentage;
    

    return steeringPercentage;
  } */
}

