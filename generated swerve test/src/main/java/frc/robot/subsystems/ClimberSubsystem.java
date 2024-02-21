// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PortConstants;

public class ClimberSubsystem extends SubsystemBase {

  TalonFX ClimberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.Slot1.kP = 10;
    climberConfig.Slot1.kI = 0;
    climberConfig.Slot1.kD = 0;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climberConfig.CurrentLimits.SupplyCurrentLimit = 10;
    FeedbackConfigs fdb = climberConfig.Feedback;
    fdb.SensorToMechanismRatio = ClimberConstants.kClimberGearRatio;

    ClimberMotor = new TalonFX(PortConstants.kClimberMotorPort);
    ClimberMotor.getConfigurator().apply(climberConfig);
    ClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    ClimberMotor.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClimberMotor() {

  }

  public void stopClimber() {
    ClimberMotor.stopMotor();
  }
}
