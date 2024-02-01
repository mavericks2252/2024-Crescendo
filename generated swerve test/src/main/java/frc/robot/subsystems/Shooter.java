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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterMotorMaster;
  TalonFX shooterMotorSlave;
  TalonFX shooterAngleMotor;
  double targetRPM = 4250;
  CANSparkMax acceleratorWheel;
  CANSparkMax feedMotor;
  CANSparkMax acceleratorWheelSlave;
  DutyCycleEncoder throughBoreEncoder;
  ProfiledPIDController autoAnglePIDController;
  VisionSubsystem vision;


  private final VelocityTorqueCurrentFOC shooterTV = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

  public Shooter() {
      TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
            shooterConfig.Slot1.kP = 9.5;
            shooterConfig.Slot1.kI = 0.6;
            shooterConfig.Slot1.kD = 0.00005;

      TalonFXConfiguration shooterAngleConfig = new TalonFXConfiguration();
            shooterAngleConfig.Slot2.kP = 0;
            shooterAngleConfig.Slot2.kI = 0;
            shooterAngleConfig.Slot2.kD = 0;

      shooterMotorMaster = new TalonFX(PortConstants.kShooterMotorMasterPort);
      shooterMotorMaster.getConfigurator().apply(shooterConfig);
      shooterMotorMaster.setNeutralMode(NeutralModeValue.Coast);
      shooterMotorMaster.setInverted(true);
 
      shooterMotorSlave = new TalonFX(PortConstants.kShooterMotorSlavePort);
      shooterMotorSlave.getConfigurator().apply(shooterConfig);
      shooterMotorSlave.setInverted(false);
      shooterMotorSlave.setControl(new Follower(PortConstants.kShooterMotorMasterPort, true));

      acceleratorWheel = new CANSparkMax(PortConstants.kAcceleratorWheelPort, MotorType.kBrushless);
      acceleratorWheel.setInverted(true);
      acceleratorWheel.setIdleMode(IdleMode.kCoast);

      acceleratorWheelSlave = new CANSparkMax(PortConstants.kacceleratorWheelSlavePort, MotorType.kBrushless);
      acceleratorWheelSlave.follow(acceleratorWheel);

      throughBoreEncoder = new DutyCycleEncoder(new DigitalInput(2));

      shooterAngleMotor = new TalonFX(PortConstants.kShooterAngleMotorPort);
      //shooterAngleMotor.getConfigurator().apply(shooterAngleConfig);
      shooterAngleMotor.setInverted(false);
      shooterAngleMotor.setNeutralMode(NeutralModeValue.Coast);

      autoAnglePIDController = new ProfiledPIDController(0, 0, 0, null, 0.1);
      autoAnglePIDController.enableContinuousInput(0, 360);
      //autoAnglePIDController.setIZone(0);


      shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
      shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;


      
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
    //SmartDashboard.putString("Shooter return type", getShooterReturnType());
    SmartDashboard.putNumber("ThroughBore Encoder", getThroughBoreEncoder());
    SmartDashboard.putNumber("poition Offset", throughBoreEncoder.getPositionOffset());
  }

  // sets shooter motor to percent speed
/*public void setPercentOutput(double topMotorSpeed, double bottomMotorSpeed){
  shooterMotorMaster.set(topMotorSpeed);
  shooterMotorSlave.set(bottomMotorSpeed);
}*/


public void acceleratorWheelOutput(double acceleratorWheelSpeed){
  acceleratorWheel.set(acceleratorWheelSpeed);
  acceleratorWheelSlave.set(acceleratorWheelSpeed);
}
public void feedMotorOutput(double feedMotorSpeed){
  feedMotor.set(feedMotorSpeed);
}

  // stops shooter motor
public  void stopShooter(){
  shooterMotorMaster.stopMotor();
  shooterMotorSlave.stopMotor();
}

public void stopAcceleratorWheel(){
  acceleratorWheel.stopMotor();
  acceleratorWheelSlave.stopMotor();
}

public double getThroughBoreEncoder(){
  return throughBoreEncoder.getAbsolutePosition() * 360;
}

  
public void stopFeedMotor(){
  feedMotor.stopMotor();
}

public void setShooterVelocity(double targetRPM){
  shooterMotorMaster.setControl(shooterTV.withVelocity(targetRPM/60));
  //shooterMotorSlave.setControl(shooterTV.withVelocity(targetRPM/60));

}

public Double getShooterVelocity(){
  double shooterVelocity = shooterMotorMaster.getVelocity().getValue();
  return shooterVelocity*60;
}

public double autoAngleOutput(){
  double currentPos = getThroughBoreEncoder();
  double targetPos = 0;

  return autoAnglePIDController.calculate(currentPos, targetPos);
}

public void setShooterAngleMotor(){
  shooterAngleMotor.set(0);
}
/*public String getShooterReturnType(){
  String shooterReturnType = shooterMotorMaster.getVelocity().getUnits();
  return shooterReturnType;
}*/



}