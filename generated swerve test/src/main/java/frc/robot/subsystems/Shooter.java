// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterMotorMaster;
  TalonFX shooterMotorSlave;
  double targetRPM;
  CANSparkMax acceleratorWheel;
  CANSparkMax acceleratorWheelSlave;
  CANSparkMax amplifierWheel;
  DigitalInput beamBreakShot;
  DigitalInput beamBreakAmp;
  DigitalInput beamBreakMiddleFront;
  DigitalInput beamBreakMiddleBack;

  private final VelocityTorqueCurrentFOC shooterTV = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

  public Shooter() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot1.kP = 8;
    shooterConfig.Slot1.kI = 0.0;
    shooterConfig.Slot1.kD = 0.1;
    shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 75;
    shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    shooterMotorMaster = new TalonFX(PortConstants.kShooterMotorMasterPort);
    shooterMotorMaster.getConfigurator().apply(shooterConfig);
    shooterMotorMaster.setNeutralMode(NeutralModeValue.Coast);
    shooterMotorMaster.setInverted(false);

    shooterMotorSlave = new TalonFX(PortConstants.kShooterMotorSlavePort);
    shooterMotorSlave.getConfigurator().apply(shooterConfig);
    // shooterMotorSlave.setControl(new
    // StrictFollower(PortConstants.kShooterMotorMasterPort));
    shooterMotorSlave.setInverted(false);

    acceleratorWheel = new CANSparkMax(PortConstants.kAcceleratorWheelPort, MotorType.kBrushless);
    acceleratorWheel.setInverted(true);
    acceleratorWheel.setIdleMode(IdleMode.kBrake);

    amplifierWheel = new CANSparkMax(PortConstants.kAmplifierWheelPort, MotorType.kBrushless);
    amplifierWheel.setInverted(false);
    amplifierWheel.setIdleMode(IdleMode.kCoast);

    beamBreakShot = new DigitalInput(PortConstants.kShotBeamBreak);
    beamBreakAmp = new DigitalInput(PortConstants.kAmpBeamBreak);
    beamBreakMiddleBack = new DigitalInput(PortConstants.kMiddleBackBeamBreak);
    beamBreakMiddleFront = new DigitalInput(PortConstants.kMiddleFrontBeamBreak);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
    SmartDashboard.putBoolean("Beambreak amp", getAmpBeambreak());
    SmartDashboard.putBoolean("BeamBreak shot", getShotBeambreak());
    SmartDashboard.putBoolean("beambreak middle front", getMiddleFrontBeambreak());
    SmartDashboard.putBoolean("Beambreak middle back", getMiddleBackBeambreak());

  }

  public void acceleratorWheelOutput(double acceleratorWheelSpeed) {
    acceleratorWheel.set(acceleratorWheelSpeed);
  }

  public void ampScore() {
    acceleratorWheel.set(-ShooterConstants.kacceleratorWheelSpeed);
    amplifierWheel.set(ShooterConstants.kacceleratorWheelSpeed);
  }

  public void intakeNote() {
    acceleratorWheel.set(.75);
    amplifierWheel.set(ShooterConstants.kIntakeSpeed);
  }

  public void setShooterVelocity(double targetRPM) {
    shooterMotorMaster.setControl(shooterTV.withVelocity(targetRPM / 60).withFeedForward(25));
    shooterMotorSlave.setControl(shooterTV.withVelocity(targetRPM / 60).withFeedForward(25));

  }

  public Double getShooterVelocity() {
    double shooterVelocity = shooterMotorMaster.getVelocity().getValue() * 60;
    return shooterVelocity;
  }

  // stops shooter motor
  public void stopShooter() {
    shooterMotorMaster.stopMotor();
    shooterMotorSlave.stopMotor();
  }

  public void stopAcceleratorWheel() {
    acceleratorWheel.stopMotor();

  }

  public void stopAmplifierWheel() {
    amplifierWheel.stopMotor();
  }

  public boolean getShotBeambreak() {
    return !beamBreakShot.get();

  }

  public boolean getAmpBeambreak() {
    return !beamBreakAmp.get();

  }

  public boolean getMiddleBackBeambreak() {
    return !beamBreakMiddleBack.get();
  }

  public boolean getMiddleFrontBeambreak() {
    return !beamBreakMiddleFront.get(); // returns true when broken
  }

}