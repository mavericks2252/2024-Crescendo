// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.VisionConstants;

public class VisionPhotonSubsystem extends SubsystemBase {
  /** Creates a new VisionPhotonSubsystem. */
    private final PhotonCamera camera;
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

    SmartDashboard.putString("photon vision pose", camera.getLatestResult().toString());

    visionEst.ifPresent(
          est -> {
            //var estPose = est.estimatedPose.toPose2d();
            //add standard deviation trust here

            //adds vision measurement to drivetrain
            drivetrain.addVisionMeasurement(
              est.estimatedPose.toPose2d(), 
              est.timestampSeconds);

              SmartDashboard.putString("photon pose", est.estimatedPose.toPose2d().toString());
              SmartDashboard.putNumber("time stamp", est.timestampSeconds);
          });
       


    
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobePose() {
    var visionEst = photonPoseEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

}
