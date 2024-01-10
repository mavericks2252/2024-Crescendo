// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  CANSparkMax intakeMotorMaster;
  DigitalInput beamBreakSensor;
 
 
  public Intake() {

    intakeMotorMaster = new CANSparkMax(PortConstants.kIntakeMotorMasterPort, MotorType.kBrushless);
    intakeMotorMaster.setIdleMode(IdleMode.kBrake);
  
    beamBreakSensor = new DigitalInput(IntakeConstants.kBeamBreak);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // sets intake motor to a percent speed
public void setPercentOutput(double speed){
  intakeMotorMaster.set(speed);
}

  // stops intake motor
public void stopIntake(){
  intakeMotorMaster.stopMotor();
}


public boolean getBeamBreak(){
  return !beamBreakSensor.get();
  
}

public void manualControl(){

  setPercentOutput(RobotContainer.m_operatorController.getRawAxis(OIConstants.kDriverYAxis));
}
 
}
