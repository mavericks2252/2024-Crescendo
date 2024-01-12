// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  TalonFX intakeMotorMaster;
  DigitalInput beamBreakSensor;
  DoubleSolenoid intakeSolenoid;
 
 
  public Intake() {

    intakeMotorMaster = new TalonFX(PortConstants.kIntakeMotorMasterPort);
    intakeMotorMaster.setNeutralMode(NeutralModeValue.Brake);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortConstants.kPneumaticForward, PortConstants.kPneumaticReverse);
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

 //opens intake
public void deployIntake(){

  intakeSolenoid.set(Value.kForward);

}
//closes intake
public void retractIntake(){

  intakeSolenoid.set(Value.kReverse);


}
 
}
