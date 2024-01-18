// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterMotorMaster;
  TalonFX shooterMotorSlave;
  double topMotorSpeed;
  double bottomMotorSpeed;
  

  public Shooter() {

      shooterMotorMaster = new TalonFX(PortConstants.kShooterMotorMasterPort);
      shooterMotorMaster.setNeutralMode(NeutralModeValue.Coast);
      shooterMotorMaster.setInverted(false);
      
      shooterMotorSlave = new TalonFX(PortConstants.kShooterMotorSlavePort);
      //shooterMotorSlave.setControl(new Follower(PortConstants.kShooterMotorMasterPort, true));
      



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // sets shooter motor to percent speed
public void setPercentOutput(double speed){
  shooterMotorMaster.set(topMotorSpeed);
  shooterMotorSlave.set(bottomMotorSpeed);
}

  // stops shooter motor
public  void stopShooter(){
  shooterMotorMaster.stopMotor();
  shooterMotorSlave.stopMotor();
}

}
