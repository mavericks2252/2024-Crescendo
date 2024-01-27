// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
  CANSparkMax feedMotorSlave;

  private final VelocityTorqueCurrentFOC shooterTV = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

  public Shooter() {

      shooterMotorMaster = new TalonFX(PortConstants.kShooterMotorMasterPort);
      shooterMotorMaster.setNeutralMode(NeutralModeValue.Coast);
      shooterMotorMaster.setInverted(true);
 
      shooterMotorSlave = new TalonFX(PortConstants.kShooterMotorSlavePort);
      shooterMotorSlave.setInverted(true);
      shooterMotorSlave.setControl(new Follower(PortConstants.kShooterMotorMasterPort, true));

      feedMotor = new CANSparkMax(PortConstants.kFeedMotorPort, MotorType.kBrushless);
      feedMotor.setInverted(true);
      feedMotor.setIdleMode(IdleMode.kCoast);

      feedMotorSlave = new CANSparkMax(PortConstants.kFeedMotorSlavePort, MotorType.kBrushless);
      feedMotorSlave.follow(feedMotor);
      feedMotorSlave.setIdleMode(IdleMode.kCoast);
      feedMotorSlave.setInverted(true);
      
      
     



      TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
      shooterConfig.Slot1.kP = 8;
      shooterConfig.Slot1.kI = 0.05;
      shooterConfig.Slot1.kD = 0.001;

      shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
      shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;


      shooterMotorMaster.getConfigurator().apply(shooterConfig);
      shooterMotorSlave.getConfigurator().apply(shooterConfig);
      
      


      



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // sets shooter motor to percent speed
/*public void setPercentOutput(double topMotorSpeed, double bottomMotorSpeed){
  shooterMotorMaster.set(topMotorSpeed);
  shooterMotorSlave.set(bottomMotorSpeed);
}*/


public void feedMotorOutput(double feedMotorSpeed){
  feedMotor.set(feedMotorSpeed);
  feedMotorSlave.set(feedMotorSpeed);
}

  // stops shooter motor
public  void stopShooter(){
  shooterMotorMaster.stopMotor();
  shooterMotorSlave.stopMotor();
}
public void stopFeedMotor(){
  feedMotor.stopMotor();
  feedMotorSlave.stopMotor();
}


public void setShooterVelocity(double topMotorRpm, double bottomMotorRpm){

  shooterMotorMaster.setControl(shooterTV.withVelocity(-bottomMotorRpm/60));
  shooterMotorSlave.setControl(shooterTV.withVelocity(topMotorRpm/60));

}

}
