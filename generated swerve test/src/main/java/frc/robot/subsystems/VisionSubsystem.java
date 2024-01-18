// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  double tx = 0;
  double ty = 0;
  double ta = 0;
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
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    
  }
}
