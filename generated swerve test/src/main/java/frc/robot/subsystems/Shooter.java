// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterMotorMaster;
  TalonFX shooterMotorSlave;
  double topMotorSpeed;
  double bottomMotorSpeed;
  CANSparkMax feedMotor;
  double feedMotorSpeed;

  public Shooter() {

      shooterMotorMaster = new TalonFX(PortConstants.kShooterMotorMasterPort);
      shooterMotorMaster.setNeutralMode(NeutralModeValue.Coast);
      shooterMotorMaster.setInverted(true);


      feedMotor = new CANSparkMax(PortConstants.kFeedMotorPort, MotorType.kBrushless);
      feedMotor.setInverted(false);
      feedMotor.setIdleMode(IdleMode.kBrake);
      
      
      shooterMotorSlave = new TalonFX(PortConstants.kShooterMotorSlavePort);
      shooterMotorSlave.setInverted(true);
      //shooterMotorSlave.setControl(new Follower(PortConstants.kShooterMotorMasterPort, true));
      



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // sets shooter motor to percent speed
public void setPercentOutput(double topMotorSpeed, double bottomMotorSpeed){
  shooterMotorMaster.set(topMotorSpeed);
  shooterMotorSlave.set(bottomMotorSpeed);
}


public void feedMotorOutput(double feedMotorSpeed){
  feedMotor.set(feedMotorSpeed);
}

  // stops shooter motor
public  void stopShooter(){
  shooterMotorMaster.stopMotor();
  shooterMotorSlave.stopMotor();
}

}
