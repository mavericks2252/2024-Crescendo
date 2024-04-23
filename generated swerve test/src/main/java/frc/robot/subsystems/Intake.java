// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PortConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  TalonFX intakeMotor;
  DigitalInput beamBreakSensor;
  CANSparkMax centeringWheelMotor;

  public Intake() {
    TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();
    intakeMotor = new TalonFX(PortConstants.kIntakeMotorPort);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    intakeMotor.getConfigurator().apply(intakeConfiguration);
    intakeMotor.setInverted(true);

    centeringWheelMotor = new CANSparkMax(PortConstants.kCenteringWheelPort, MotorType.kBrushless);
    centeringWheelMotor.restoreFactoryDefaults();
    centeringWheelMotor.setIdleMode(IdleMode.kBrake);
    centeringWheelMotor.setInverted(false);
    centeringWheelMotor.setSmartCurrentLimit(20);

    beamBreakSensor = new DigitalInput(IntakeConstants.kBeamBreak);
    SmartDashboard.putNumber("intake motor speed", intakeMotor.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake beambreak", getBeamBreak());
  }

  // sets intake motor to a percent speed
  public void setIntakeSpeed() { // sets the speed of our intake
    intakeMotor.set(IntakeConstants.kIntakeSpeed); // sets intake to a specified speed
    centeringWheelMotor.set(IntakeConstants.kCenteringWheelSpeed); // sets centering wheels to a specified speed
  }

  public void setIntakeBackwards() {
    intakeMotor.set(-0.5); // runs the intake motor backwards
    centeringWheelMotor.set(-0.5); // runs the centering wheels backwards
  }

  public void stopIntake() { // stops the intake motors
    intakeMotor.stopMotor(); // stops intake motor
    centeringWheelMotor.stopMotor(); // stops centering wheels
  }

  public boolean getBeamBreak() {
    return !beamBreakSensor.get(); // returns true if the beam is broken

  }

}