// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;



public class AutoAimSubsystem extends SubsystemBase {
  TrapezoidProfile.Constraints aim_PIDConstraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate, TunerConstants.kMaxAngularAcceleration);
  
  
  ProfiledPIDController autoAimPIDController;
  ProfiledPIDController noteAimPidController;
  //VisionSubsystem vision;
  VisionPhotonSubsystem photon;

  /** Creates a new AutoAimSubsystem. */
  public AutoAimSubsystem(VisionPhotonSubsystem photon, LEDSubsystem ledSubsystem) {
    //this.vision = vision;
    this.photon = photon;
    
      
    noteAimPidController = new ProfiledPIDController(5, 0.25, 0, aim_PIDConstraints, .01);
    noteAimPidController.enableContinuousInput(-Math.PI, Math.PI);
    noteAimPidController.setTolerance(Units.degreesToRadians(1));
    
    
    autoAimPIDController= new ProfiledPIDController(5, 0.25, 0, aim_PIDConstraints,.01);
    autoAimPIDController.enableContinuousInput(-Math.PI, Math.PI);
    
    //autoAimPIDController.setTolerance(3);
    autoAimPIDController.setIZone(.5);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("auto aim speaker output", speakerAutoAimRateOutput());
    SmartDashboard.putNumber("auto aim note output", noteAutoAimRateOutput());
    
  }

  public double speakerAutoAimRateOutput(){
    Pose2d currentPos = photon.getCurrentPose2d();
    Pose2d targetPos = photon.getSpeakerTargetRotation2d();
  
    return autoAimPIDController.calculate(currentPos.getRotation().getRadians(), targetPos.getRotation().getRadians());

   }

   public double noteAutoAimRateOutput(){
    //double currentRotation = photon.getCurrentPose2d().getRotation().getDegrees();
    double angleToNote;
    double turnRate;
    var target = photon.camera.getLatestResult();

    if(target.hasTargets()){
      angleToNote = target.getBestTarget().getYaw();
      turnRate = noteAimPidController.calculate(Units.degreesToRadians(angleToNote));
    }
    else{
      angleToNote = 0;
      turnRate = CommandSwerveDrivetrain.getExponential(-RobotContainer.m_driver_controler.getRightX() * RobotContainer.MaxAngularRate);
    }
    SmartDashboard.putNumber("angle to note", turnRate);
    return turnRate;
    
    
   }

}
