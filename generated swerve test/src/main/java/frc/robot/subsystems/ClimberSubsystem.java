// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PortConstants;

public class ClimberSubsystem extends SubsystemBase {

  TalonFX climberMotor;
  private final PositionDutyCycle climberPosition = new PositionDutyCycle(0,
      0,
      false,
      0,
      1,
      false,
      false,
      false);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    climberConfig.Slot1.kP = 10;
    climberConfig.Slot1.kI = 0;
    climberConfig.Slot1.kD = 0;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2.979;
    climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climberConfig.CurrentLimits.SupplyCurrentLimit = 40;
    FeedbackConfigs fdb = climberConfig.Feedback;
    fdb.SensorToMechanismRatio = ClimberConstants.kClimberGearRatio;

    climberMotor = new TalonFX(PortConstants.kClimberMotorPort);
    climberMotor.getConfigurator().apply(climberConfig);
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
    climberMotor.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command applyRequests(Supplier<Double> speed) {

    return run(() -> this.setClimberMotor(speed.get()));
  }

  public void setClimberMotor(double speed) {
    if (Math.abs(speed) < .1) {
      speed = 0;
    }
    SmartDashboard.putNumber("climber speed", speed);
    climberMotor.set(speed);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }

  public void climberup() {
    climberMotor.setControl(climberPosition.withPosition(31));
  }

}
