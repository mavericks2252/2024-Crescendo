// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

    intakeMotor = new TalonFX(PortConstants.kIntakeMotorPort);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setInverted(true);

    centeringWheelMotor = new CANSparkMax(PortConstants.kCenteringWheelPort, MotorType.kBrushless);
    centeringWheelMotor.restoreFactoryDefaults();
    centeringWheelMotor.setIdleMode(IdleMode.kBrake);
    centeringWheelMotor.setInverted(false);
    centeringWheelMotor.setSmartCurrentLimit(20);

    beamBreakSensor = new DigitalInput(IntakeConstants.kBeamBreak);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("intake beambreak", getBeamBreak());
  }

  // sets intake motor to a percent speed
  public void setIntakeSpeed() { // sets the speed of our intake
    intakeMotor.set(IntakeConstants.kIntakeSpeed);
    centeringWheelMotor.set(IntakeConstants.kCenteringWheelSpeed);
  }

  public void setIntakeBackwards() {
    intakeMotor.set(-IntakeConstants.kIntakeSpeed);
    centeringWheelMotor.set(-IntakeConstants.kCenteringWheelSpeed);
  }

  public void stopIntake() { // stops the intake motors
    intakeMotor.stopMotor();
    centeringWheelMotor.stopMotor();
  }

  public boolean getBeamBreak() {
    return !beamBreakSensor.get(); // returns true if the beam is broken

  }

}