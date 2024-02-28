// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  double lastEstTimestamp = 0;
  public final PhotonCamera noteCam;

  CommandSwerveDrivetrain drivetrain;

  TrapezoidProfile.Constraints aim_PIDConstraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate,
      TunerConstants.kMaxAngularAcceleration);

  ProfiledPIDController autoAimPIDController;
  ProfiledPIDController noteAimPidController;

  public VisionPhotonSubsystem(CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;
    camera = new PhotonCamera(VisionConstants.kCameraName);
    noteCam = new PhotonCamera(VisionConstants.kNoteCameraName);
    photonPoseEstimator = new PhotonPoseEstimator(
        VisionConstants.kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        VisionConstants.kRobotToCam);

    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    noteAimPidController = new ProfiledPIDController(5, 0.25, 0, aim_PIDConstraints, .01);
    noteAimPidController.enableContinuousInput(-Math.PI, Math.PI);
    noteAimPidController.setTolerance(Units.degreesToRadians(1));

    autoAimPIDController = new ProfiledPIDController(15, 0.25, 0, aim_PIDConstraints, .01);
    autoAimPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // autoAimPIDController.setTolerance(3);
    autoAimPIDController.setIZone(.5);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var visionEst = getEstimatedGlobePose();

    visionEst.ifPresent( // if we have an estimated pose
        est -> {
          // var estPose = est.estimatedPose.toPose2d();
          // add standard deviation trust here

          double targetArea = camera.getCameraTable().getValue("targetArea").getDouble();
          // adds vision measurement to drivetrain
          if (targetArea > 0.04) {
            drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds);
          }

          SmartDashboard.putString("photon pose", est.estimatedPose.toPose2d().toString());
        });
    SmartDashboard.putNumber("autoSpeakerAimOutput", speakerAutoAimRateOutput());
    SmartDashboard.putNumber("speakerDistance", getSpeakerDistance());
    SmartDashboard.putNumber("Target Angle", getTargetAngle());
    SmartDashboard.putNumber("Amp Distance", getAmpDistance());
    SmartDashboard.putString("current Bot pose", getCurrentPose2d().toString());
    // noteAutoAimRateOutput();

  }

  public Optional<EstimatedRobotPose> getEstimatedGlobePose() { // will return an estimated pose but only when it has
                                                                // one
    var visionEst = photonPoseEstimator.update(); // recieves the newest pose estimation
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds(); // gets the time that it took the latest
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

  public boolean driverStationAlliance() {
    var alliance = DriverStation.getAlliance(); // creates a variable called alliance
    if (alliance.isPresent()) { // checks if alliance variable has been set to anything
      return alliance.get() == DriverStation.Alliance.Red; // checks if the alliance color is red
    }
    return false; // returns false if color is not red or if no color
  }

  public double getSpeakerDistance() {

    double speakerYPos;

    if (driverStationAlliance()) {
      speakerYPos = FieldConstants.kRedSpeakerYPosMeters; // sets the robot to look for the red speaker if on red team
    } else {
      speakerYPos = FieldConstants.kBlueSpeakerYPosMeters; // sets the robot to look for the blue speaker if on blue
                                                           // team
    }

    double botX = drivetrain.getState().Pose.getX(); // gets the x pose of the bot
    double botY = drivetrain.getState().Pose.getY(); // gets the y pose of the bot
    double oppositetSide = botY - speakerYPos; // finds the distance from the bot to the back of the arena

    return Math.hypot(oppositetSide, botX); // calculates the distance with an angle to the speaker
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
    else if (distance > 4) { // if the distance is between 4 and 5.5 meters
      targetAngle = -3.1667 * distance + 129.67; // use this equasion
    }
    // close shot less than 4 meters
    else { // if the distance is less than 4 meters
      targetAngle = -8.9629 * distance + 152.85; // use this equasion
    }

    return targetAngle;
    // return SmartDashboard.getNumber("TestShooterAngle", 112.65);
  }

  public double getAmpDistance() {
    double botX = drivetrain.getState().Pose.getX(); // get the x position of the bot
    double botY = drivetrain.getState().Pose.getY(); // gets the y position of the bot
    double oppositetSide = botY - FieldConstants.kBlueAmpYPosMeters; // finds how far the bot y is from the amp y
    double adjacentSide = botX - FieldConstants.kBlueAmpXPosMeters; // finds how far the bot x is from the amp x

    return Math.hypot(oppositetSide, adjacentSide) - 1; // calculates the distance from amp with an offset to account
                                                        // for robot size and bumpers
  }

  public void seedRobotPoseFromVision() {
    drivetrain.seedFieldRelative(); // seeds the robot when it sees a tag

  }

  public void setPhotonPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline); // sets the photon vision pipeline to any we choose
  }

  public double speakerAutoAimRateOutput() {
    Pose2d currentPos = getCurrentPose2d(); // gets the current pose of the bot
    Pose2d targetPos = getSpeakerTargetRotation2d();
    // double moveCorrection = 3 * -RobotContainer.m_driver_controler.getLeftX();
    double correction = Units.degreesToRadians(1);
    double targetAngle = targetPos.getRotation().getRadians() - correction; // gets the rotation needed to reach the
                                                                            // speaker

    return autoAimPIDController.calculate(currentPos.getRotation().getRadians(), targetAngle);

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
          .getExponential(-RobotContainer.m_driver_controler.getRightX() * RobotContainer.MaxAngularRate);
    }

    return turnRate;

  }

  public double getNoteTargetArea() {
    var target = noteCam.getLatestResult();
    if (target.hasTargets())
      return noteCam.getCameraTable().getValue("targetArea").getDouble();
    else
      return 0;
  }

}
