// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.generated.TunerConstants;

public class ShooterRotationSubsystem extends SubsystemBase {
  DutyCycleEncoder throughBoreEncoder;
  ProfiledPIDController autoAnglePIDController;
  TalonFX shooterAngleMotor;
  //VisionSubsystem vision;
  VisionPhotonSubsystem photon;

  TrapezoidProfile.Constraints angle_PIDConstraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate, TunerConstants.kMaxAngularAcceleration);

  /** Creates a new ShooterRotationSubsystem. */
  public ShooterRotationSubsystem(VisionPhotonSubsystem photon) {
    //this.vision = vision;
    this.photon = photon;
    TalonFXConfiguration shooterAngleConfig = new TalonFXConfiguration();
            shooterAngleConfig.Slot2.kP = 0;
            shooterAngleConfig.Slot2.kI = 0;
            shooterAngleConfig.Slot2.kD = 0;




      shooterAngleMotor = new TalonFX(PortConstants.kShooterAngleMotorPort);
      shooterAngleMotor.getConfigurator().apply(shooterAngleConfig);
      shooterAngleMotor.setInverted(false);
      shooterAngleMotor.setNeutralMode(NeutralModeValue.Coast);

      throughBoreEncoder = new DutyCycleEncoder(new DigitalInput(2));


      autoAnglePIDController = new ProfiledPIDController(2, 0.25, 0, angle_PIDConstraints, 0.01);
      autoAnglePIDController.enableContinuousInput(0, 360);
      //autoAnglePIDController.setIZone(0);

      
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("poition Offset", throughBoreEncoder.getPositionOffset());
   
    SmartDashboard.putNumber("ThroughBore Encoder", getThroughBoreEncoder());
    SmartDashboard.putNumber("auto Angle output", getAutoAngleOutput());


  }

  public double getThroughBoreEncoder(){
    return throughBoreEncoder.getAbsolutePosition() * 360;
  }

  public double getAutoAngleOutput(){
    double currentPos = getThroughBoreEncoder();
    double targetPos = photon.getTargetAngle();
  
    return autoAnglePIDController.calculate(currentPos, targetPos);
  }
  
  public void setShooterAngleMotor(double angleSpeed){
    shooterAngleMotor.set(angleSpeed);
  }

}
