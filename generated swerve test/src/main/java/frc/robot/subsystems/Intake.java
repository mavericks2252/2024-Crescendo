// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PortConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  CANSparkMax intakeMotorMaster;
  CANSparkMax intakeMotorSlave;
  DigitalInput beamBreakSensor;
  DoubleSolenoid intakeSolenoid;
  double motorMasterSpeed;
  double motorSlaveSpeed;
 
 
  public Intake() {

    intakeMotorMaster = new CANSparkMax(PortConstants.kIntakeMotorMasterPort, MotorType.kBrushless);
    intakeMotorMaster.setIdleMode(IdleMode.kBrake);
    intakeMotorMaster.setInverted(false);
    
    intakeMotorSlave = new CANSparkMax(PortConstants.kIntakeMotorSlavePort, MotorType.kBrushless);
    intakeMotorSlave.setIdleMode(IdleMode.kBrake);
    intakeMotorSlave.setInverted(false);

    intakeMotorSlave.follow(intakeMotorMaster);
    
    
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortConstants.kPneumaticForward, PortConstants.kPneumaticReverse);
    
    
    beamBreakSensor = new DigitalInput(IntakeConstants.kBeamBreak);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // sets intake motor to a percent speed
public void setPercentOutput(double motorMasterSpeed, double motorSlaveSpeed){
  intakeMotorMaster.set(motorMasterSpeed);
  intakeMotorSlave.set(motorSlaveSpeed);
}

  // stops intake motor
public void stopIntake(){
  intakeMotorMaster.stopMotor();
}


public boolean getBeamBreak(){
  return !beamBreakSensor.get();
  
}


 //opens intake
public void deployIntake(){

  intakeSolenoid.set(Value.kForward);

}
//closes intake
public void retractIntake(){

  intakeSolenoid.set(Value.kReverse);


}
 
}
