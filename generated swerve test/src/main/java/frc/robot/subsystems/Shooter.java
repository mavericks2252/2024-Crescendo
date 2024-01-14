// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterMotorMaster;
  TalonFX shooterMotorSlave;
  
  public Shooter() {

      shooterMotorMaster = new TalonFX(PortConstants.kShooterMotorMasterPort);
      shooterMotorMaster.setNeutralMode(NeutralModeValue.Coast);
      shooterMotorMaster.setInverted(true);
      
      shooterMotorSlave = new TalonFX(PortConstants.kShooterMotorSlavePort);
      shooterMotorSlave.setControl(new Follower(PortConstants.kShooterMotorMasterPort, true));



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
