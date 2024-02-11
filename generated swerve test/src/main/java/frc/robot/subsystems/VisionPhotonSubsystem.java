// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;


public class VisionPhotonSubsystem extends SubsystemBase {
  /** Creates a new VisionPhotonSubsystem. */
    public final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    double lastEstTimestamp = 0;
    CommandSwerveDrivetrain drivetrain;

  public VisionPhotonSubsystem(CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;
    camera = new PhotonCamera(VisionConstants.kCameraName);
    photonPoseEstimator = new PhotonPoseEstimator(
      VisionConstants.kTagLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      camera,
      VisionConstants.kRobotToCam);
    
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    var visionEst = getEstimatedGlobePose();

    visionEst.ifPresent(
          est -> {
            //var estPose = est.estimatedPose.toPose2d();
            //add standard deviation trust here

            //adds vision measurement to drivetrain
            drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(), 
              est.timestampSeconds);

              SmartDashboard.putString("photon pose", est.estimatedPose.toPose2d().toString());
             });
       
   

    SmartDashboard.putNumber("speakerDistance", getSpeakerDistance());
    SmartDashboard.putNumber("Target Angle", getTargetAngle());

    SmartDashboard.putNumber("Amp Distance", getAmpDistance());
    SmartDashboard.putString("current Bot pose", getCurrentPose2d().toString());
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobePose() {
    var visionEst = photonPoseEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }








  public Pose2d getCurrentPose2d(){
    return drivetrain.getState().Pose; //sets the method getCurrentPose2d to the value of the current pose
  }

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

    // return a pose 2d of robot location and target angle of speaker
    return new Pose2d(xRobotPosMeters, yRobotPosMeters, (new Rotation2d(speakerPos.getX() - xRobotPosMeters, speakerPos.getY() - yRobotPosMeters))); //finds the pose of the robot using the location and angle of the speaker
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

  
  public void seedRobotPoseFromVision(){
    drivetrain.seedFieldRelative(); //seeds the robot when it sees a tag

  }

  public void setPhotonPipeline(int pipeline){
    camera.setPipelineIndex(pipeline);
  }

 
}

