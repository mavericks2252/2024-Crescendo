// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;



public class AutoAimSubsystem extends SubsystemBase {
  TrapezoidProfile.Constraints aim_PIDConstraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate, TunerConstants.kMaxAngularAcceleration);
  

  ProfiledPIDController autoAimPIDController;
  VisionSubsystem vision;

  /** Creates a new AutoAimSubsystem. */
  public AutoAimSubsystem(VisionSubsystem vision) {
    this.vision = vision;
    
      
      
    autoAimPIDController= new ProfiledPIDController(4, 0.25, 0, aim_PIDConstraints,.01);
    autoAimPIDController.enableContinuousInput(-Math.PI, Math.PI);
    
    //autoAimPIDController.setTolerance(3);
    autoAimPIDController.setIZone(.4);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("auto aim output", autoAimRateOutput());
    
  }

  public double autoAimRateOutput(){
    Pose2d currentPos = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    Pose2d targetPos = vision.getSpeakerTargetRotation2d(FieldConstants.kBlueSpeaker);
  
    return LimelightHelpers.getTV("limelight") ? 
              autoAimPIDController.calculate(currentPos.getRotation().getRadians(), targetPos.getRotation().getRadians()) : 0;

   }

}
