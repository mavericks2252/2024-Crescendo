// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class VisionPhotonSubsystem extends SubsystemBase {
  /** Creates a new VisionPhotonSubsystem. */
  public final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonPoseEstimator backLeftPoseEstimator;
  private final PhotonPoseEstimator backRightPoseEstimator;
  double lastEstTimestamp = 0;
  public final PhotonCamera noteCam;
  public final PhotonCamera backLeftAprilTagCam;
  public final PhotonCamera backRightAprilTagCam;
  public final InterpolatingDoubleTreeMap shooterMap;

  CommandSwerveDrivetrain drivetrain;

  TrapezoidProfile.Constraints aim_PIDConstraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate,
      TunerConstants.kMaxAngularAcceleration);

  ProfiledPIDController autoAimPIDController;
  ProfiledPIDController noteAimPidController;

  public VisionPhotonSubsystem(CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;
    camera = new PhotonCamera(VisionConstants.kCameraName);
    noteCam = new PhotonCamera(VisionConstants.kNoteCameraName);
    backLeftAprilTagCam = new PhotonCamera(VisionConstants.kTagCameraBL);
    backRightAprilTagCam = new PhotonCamera(VisionConstants.kTagCameraBR);
    photonPoseEstimator = new PhotonPoseEstimator(
        VisionConstants.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        VisionConstants.kRobotToCam);

    backLeftPoseEstimator = new PhotonPoseEstimator(
        VisionConstants.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        backLeftAprilTagCam,
        VisionConstants.kRobotToBackLeftCam);

    backRightPoseEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        backRightAprilTagCam,
        VisionConstants.kRobotToBackRightCam);

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    backLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    backRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    noteAimPidController = new ProfiledPIDController(3, 0.25, 0, aim_PIDConstraints, .01);
    noteAimPidController.enableContinuousInput(-Math.PI, Math.PI);
    noteAimPidController.setTolerance(Units.degreesToRadians(1));

    autoAimPIDController = new ProfiledPIDController(10, 0.25, 0.25, aim_PIDConstraints, .01);
    autoAimPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // autoAimPIDController.setTolerance(3);
    autoAimPIDController.setIZone(.5);

    shooterMap = new InterpolatingDoubleTreeMap();
    createShooterMap();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    addPhotonVisionMeasurement(camera, photonPoseEstimator, "front cam pose");
    addPhotonVisionMeasurement(backLeftAprilTagCam, backLeftPoseEstimator, "back left pose");
    addPhotonVisionMeasurement(backRightAprilTagCam, backRightPoseEstimator, "back right pose");

    // SmartDashboard.putNumber("autoSpeakerAimOutput", speakerAutoAimRateOutput());
    SmartDashboard.putNumber("speakerDistance", getSpeakerDistance());
    SmartDashboard.putNumber("Target Angle", getTargetAngle());
    SmartDashboard.putNumber("Amp Distance", getAmpDistance());
    SmartDashboard.putString("current Bot pose", getCurrentPose2d().toString());
    // noteAutoAimRateOutput();

  }

  public void addPhotonVisionMeasurement(PhotonCamera camera, PhotonPoseEstimator poseEstimator, String photonPose) {
    var visionEst = getEstimatedGlobePose(camera, poseEstimator);

    visionEst.ifPresent( // if we have an estimated pose
        est -> {
          // var estPose = est.estimatedPose.toPose2d();
          // add standard deviation trust here

          double targetArea = camera.getCameraTable().getValue("targetArea").getDouble();
          // adds vision measurement to drivetrain
          double errorOfEstimatedPose = PhotonUtils.getDistanceToPose(drivetrain.getState().Pose,
              est.estimatedPose.toPose2d());

          if (targetArea > 0.04 && errorOfEstimatedPose < 1) {
            drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds);
            SmartDashboard.putBoolean("is seeded", true);
          } else
            SmartDashboard.putBoolean("is seeded", false);

          SmartDashboard.putString(photonPose, poseEstimator.toString());
        });
  }

  /**
   * This is the description of what the method does
   * 
   * @param photonCamera  this is what this parameter is that the method is
   *                      requesting
   * @param poseEstimator this is what poseEstimator is
   * @return this is a description of what the method returns
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobePose(PhotonCamera photonCamera,
      PhotonPoseEstimator poseEstimator) { // will return an estimated pose but only when it has
    // one
    var visionEst = poseEstimator.update(); // recieves the newest pose estimation
    double latestTimestamp = photonCamera.getLatestResult().getTimestampSeconds(); // gets the time that it took the
                                                                                   // latest
    // pose estimation

    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5; // sees if the time of this estimation is
                                                                             // too close to the time of the last
                                                                             // estimation
    if (newResult) // if the time isn't too close
      lastEstTimestamp = latestTimestamp; // sets our current pose to our old pose
    return visionEst; // returns the new pose
  }

  public Pose2d getCurrentPose2d() {
    return drivetrain.getState().Pose; // sets the method getCurrentPose2d to the value of the current pose
  }

  public Pose2d getSpeakerTargetRotation2d() {
    double xRobotPosMeters = getCurrentPose2d().getX(); // gets the x pose of the bot
    double yRobotPosMeters = getCurrentPose2d().getY(); // gets the y pose of the bot
    Translation2d speakerPos; // gets the position of the speaker

    if (driverStationAlliance()) {
      speakerPos = FieldConstants.kRedSpeaker; // sets the location of the speaker for the red side

    }

    else {
      speakerPos = FieldConstants.kBlueSpeaker; // sets the location of the speaker for the blue side

    }

    // return a pose 2d of robot location and target angle of speaker
    return new Pose2d(xRobotPosMeters, yRobotPosMeters,
        (new Rotation2d(speakerPos.getX() - xRobotPosMeters, speakerPos.getY() - yRobotPosMeters))); // finds the pose
                                                                                                     // of the robot
                                                                                                     // using the
                                                                                                     // location and
                                                                                                     // angle of the
                                                                                                     // speaker
  }

  public Pose2d getCornerTargetRotation2d() {
    double xRobotPosMeters = getCurrentPose2d().getX(); // gets the x pose of the bot
    double yRobotPosMeters = getCurrentPose2d().getY(); // gets the y pose of the bot
    Translation2d cornerPos; // gets the position of the speaker

    if (driverStationAlliance()) {
      cornerPos = FieldConstants.kAmpCornerRed; // sets the location of the speaker for the red side

    }

    else {
      cornerPos = FieldConstants.kAmpCornerBlue; // sets the location of the speaker for the blue side

    }

    // return a pose 2d of robot location and target angle of speaker
    return new Pose2d(xRobotPosMeters, yRobotPosMeters,
        (new Rotation2d(cornerPos.getX() - xRobotPosMeters, cornerPos.getY() - yRobotPosMeters))); // finds the pose
                                                                                                   // of the robot
                                                                                                   // using the
                                                                                                   // location and
                                                                                                   // angle of the
                                                                                                   // speaker
  }

  public double cornerAutoAimRateOutput() {
    Pose2d currentPos = getCurrentPose2d(); // gets the current pose of the bot
    Pose2d targetPos = getCornerTargetRotation2d();
    // double moveCorrection = 3 * -RobotContainer.m_driver_controler.getLeftX();
    double targetAngle = targetPos.getRotation().getRadians(); // gets the rotation needed to reach the
                                                               // speaker

    double output = autoAimPIDController.calculate(currentPos.getRotation().getRadians(), targetAngle);

    return output;

  }

  public boolean driverStationAlliance() {
    var alliance = DriverStation.getAlliance(); // creates a variable called alliance
    if (alliance.isPresent()) { // checks if alliance variable has been set to anything
      return alliance.get() == DriverStation.Alliance.Red; // checks if the alliance color is red
    }
    return false; // returns false if color is not red or if no color
  }

  public double getSpeakerDistance() {

    double speakerXPos;
    double speakerYPos;

    if (driverStationAlliance()) {
      speakerXPos = FieldConstants.kRedSpeakerXPosMeters; // sets the robot to look for the red speaker if on red team
      speakerYPos = FieldConstants.kRedSpeakerYPosMeters;
    } else {
      speakerXPos = FieldConstants.kBlueSpeakerXPosMeters; // sets the robot to look for the blue speaker if on blue
      speakerYPos = FieldConstants.kBlueSpeakerYPosMeters;
      // team
    }

    double botX = drivetrain.getState().Pose.getX(); // gets the x pose of the bot
    double botY = drivetrain.getState().Pose.getY(); // gets the y pose of the bot
    double oppositetSide = botX - speakerXPos; // finds the distance from the bot to the back of the arena
    double adjacentSide = botY - speakerYPos;

    return Math.abs(Math.hypot(oppositetSide, adjacentSide)); // calculates the distance with an angle to the speaker
    // return Math.sqrt((botX*botX)+(oppositetSide*oppositetSide));

  }

  public double getTargetRPM() {

    double distance = getSpeakerDistance();

    if (distance < 4)
      return 426.8 * distance + 1792.8; // if the bot is within 4 meters of the speaker use this equasion

    else// (distance < 5.5) // if the bot is within 5.5 to 4 meters of the speaker
      return 3500; // shoot at 3500 rpm
    /*
     * else // if it is not within 5.5
     * return 125 * distance + 2812.5; // shoot based on this equasion
     */

    // return SmartDashboard.getNumber("test shooter RPM", 0);
  }

  public double getTargetAngle() {

    double targetAngle;
    double distance = getSpeakerDistance();
    // far shot farther than 5.5 meters
    if (distance > 5.5) { // if the distance to the speakeris more than 5.5 meters
      targetAngle = -2.25 * distance + 124.63; // use this equasion
    }
    // middle shot 5.5 meters to 4 meters
    else if (distance > 3.5) { // if the distance is between 4 and 5.5 meters
      targetAngle = -3.125 * distance + 129.94; // use this equasion
    }
    // close shot less than 4 meters
    else { // if the distance is less than 4 meters
      targetAngle = -10.309 * distance + 155.08; // use this equasion
    }

    return targetAngle;
    // return SmartDashboard.getNumber("TestShooterAngle", 112.65);
  }

  private double getMapTargetAngle() {
    return shooterMap.get(getSpeakerDistance());
  }

  private void createShooterMap() {
    // Create shooter Map
    // distance angle
    shooterMap.put(5.5, 109.0);
    shooterMap.put(4.0, 112.0);
    shooterMap.put(3.5, 115.0);

  }

  public double getAmpDistance() {
    double ampXPos;
    if (driverStationAlliance()) {
      ampXPos = FieldConstants.kRedAmpXPosMeters;
    } else {
      ampXPos = FieldConstants.kBlueAmpXPosMeters;
    }
    double botX = drivetrain.getState().Pose.getX(); // get the x position of the bot
    double botY = drivetrain.getState().Pose.getY(); // gets the y position of the bot
    double oppositetSide = botY - FieldConstants.kBlueAmpYPosMeters; // finds how far the bot y is from the amp y
    double adjacentSide = botX - ampXPos; // finds how far the bot x is from the amp x

    return Math.hypot(oppositetSide, adjacentSide) - 1; // calculates the distance from amp with an offset to account
                                                        // for robot size and bumpers
  }

  public double getAmpDistanceEasy() {
    Pose2d ampPose;
    if (driverStationAlliance()) {
      ampPose = FieldConstants.kRedAmp;
    } else {
      ampPose = FieldConstants.kBlueAmp;
    }

    return PhotonUtils.getDistanceToPose(drivetrain.getState().Pose, ampPose);

  }

  public void seedRobotPoseFromVision() {
    drivetrain.seedFieldRelative(); // seeds the robot when it sees a tag

  }

  public void setPhotonPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline); // sets the photon vision pipeline to any we choose
  }

  public double speakerAutoAimRateOutput() {
    double correctionValue;
    if (drivetrain.getState().Pose.getY() < 4)
      correctionValue = 4;
    else
      correctionValue = 3;
    Pose2d currentPos = getCurrentPose2d(); // gets the current pose of the bot
    Pose2d targetPos = getSpeakerTargetRotation2d();
    // double moveCorrection = 3 * -RobotContainer.m_driver_controler.getLeftX();
    double correction = Units.degreesToRadians(correctionValue);
    double targetAngle = targetPos.getRotation().getRadians() - correction; // gets the rotation needed to reach the
                                                                            // speaker

    double output = autoAimPIDController.calculate(currentPos.getRotation().getRadians(), targetAngle);
    SmartDashboard.putNumber("correction", correctionValue);

    /*
     * double sign = Math.signum(output);
     * double absOutput = Math.abs(output);
     * if (0.4 > absOutput && absOutput > 0.075)
     * output = 0.4 * sign;
     */
    return output;

  }

  public double noteAutoAimRateOutput() {
    // double currentRotation =
    // photon.getCurrentPose2d().getRotation().getDegrees();
    double angleToNote;
    double turnRate;
    var target = noteCam.getLatestResult(); // gets the latest pose from the camera

    if (target.hasTargets()) { // if the camera had targets
      angleToNote = target.getBestTarget().getYaw(); // find the best target and get the angle to it
      turnRate = noteAimPidController.calculate(Units.degreesToRadians(angleToNote)); // calculates how far the robot
                                                                                      // needs to turn to the note in
                                                                                      // radians
    } else { // if there were no notes detected
      angleToNote = 0; // set the angle needed to get to a note to 0
      turnRate = CommandSwerveDrivetrain // sets the turning of the robot to the controller
          .getRotationalExponential(-RobotContainer.m_driver_controler.getRightX() * RobotContainer.MaxAngularRate);
    }

    return turnRate;

  }

  public double getNoteTargetArea() {
    var target = noteCam.getLatestResult(); // gets information on the latest camera result
    if (target.hasTargets()) // if the camera had a target
      return noteCam.getCameraTable().getValue("targetArea").getDouble(); // find the area of the target
    else
      return 0; // if not, return nothing
  }

  public Pose2d getCornerTargetRotation2dEasy() {
    Pose2d cornerPos; // gets the position of the speaker

    if (driverStationAlliance()) {

      // why is this a Translation2d??????
      // We should always make things a Pose2d in Constants
      cornerPos = new Pose2d(FieldConstants.kAmpCornerRed, new Rotation2d()); // sets the location of the speaker for
                                                                              // the red side
    } else {
      cornerPos = new Pose2d(FieldConstants.kAmpCornerBlue, new Rotation2d()); // sets the location of the speaker for
                                                                               // the blue side
    }

    Pose2d currenPose2d = getCurrentPose2d();
    Rotation2d yawToCorner = PhotonUtils.getYawToPose(currenPose2d, cornerPos);
    return currenPose2d.rotateBy(yawToCorner);

  }
}
