// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax shooterMotorMaster;
  public Shooter() {

      shooterMotorMaster = new CANSparkMax(PortConstants.kShooterMotorMasterPort, MotorType.kBrushless);
      shooterMotorMaster.setIdleMode(IdleMode.kCoast);




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // sets shooter motor to percent speed
public void setPercentOutput(double speed){
  shooterMotorMaster.set(speed);
}

  // stops shooter motor
public  void stopShooter(){
  shooterMotorMaster.stopMotor();
}

}
