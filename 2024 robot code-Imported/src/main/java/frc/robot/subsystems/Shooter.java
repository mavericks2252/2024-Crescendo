// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
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
  public Shooter() {

      shooterMotorMaster = new TalonFX(14);
      shooterMotorMaster.setNeutralMode(NeutralModeValue.Brake);




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
