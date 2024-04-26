// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterMotorMaster;
  TalonFX shooterMotorSlave;
  CANSparkMax acceleratorWheel;
  CANSparkMax acceleratorWheelSlave;
  CANSparkMax amplifierWheel;
  DigitalInput beamBreakShot;
  DigitalInput beamBreakAmp;
  DigitalInput beamBreakMiddleFront;
  DigitalInput beamBreakMiddleBack;
  TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
  TimeOfFlight frontTOF;
  ShuffleboardTab shooterTab = Shuffleboard.getTab("shooter");
  double test = 0;

  private final VelocityTorqueCurrentFOC shooterTV = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

  public Shooter() {
    shooterConfig.Slot1.kP = 8;
    shooterConfig.Slot1.kI = 0.0;
    shooterConfig.Slot1.kD = 0.1;
    shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 65;
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
    acceleratorWheel.setSmartCurrentLimit(40);

    amplifierWheel = new CANSparkMax(PortConstants.kAmplifierWheelPort, MotorType.kBrushless);
    amplifierWheel.setInverted(false);
    amplifierWheel.setIdleMode(IdleMode.kCoast);
    amplifierWheel.setSmartCurrentLimit(40);

    beamBreakShot = new DigitalInput(PortConstants.kShotBeamBreak);
    beamBreakAmp = new DigitalInput(PortConstants.kAmpBeamBreak);
    beamBreakMiddleBack = new DigitalInput(PortConstants.kMiddleBackBeamBreak);
    beamBreakMiddleFront = new DigitalInput(PortConstants.kMiddleFrontBeamBreak);

    frontTOF = new TimeOfFlight(25);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
    SmartDashboard.putBoolean("Beambreak amp", getAmpBeambreak());
    SmartDashboard.putBoolean("BeamBreak shot", getShotBeambreak());
    SmartDashboard.putBoolean("beambreak middle front", getMiddleFrontBeambreak());
    SmartDashboard.putBoolean("Beambreak middle back", getMiddleBackBeambreak());
    if (getMiddleBackBeambreak() || getMiddleFrontBeambreak())
      LEDSubsystem.green();

    // SmartDashboard.putNumber("torqueCurrent",
    // shooterMotorMaster.getTorqueCurrent().getValue());
    SmartDashboard.putNumber("accelerator wheel output", acceleratorWheel.get());

    shooterTab.add(beamBreakAmp);
    shooterTab.addNumber("test", () -> test);
    shooterTab.addBoolean("test 2", () -> getAmpBeambreak());
    shooterTab.add(shooterMotorMaster);

  }

  public void acceleratorWheelOutput(double acceleratorWheelSpeed) {
    acceleratorWheel.set(acceleratorWheelSpeed); // starts the accelerator wheels
  }

  public void ampScore() { // runs the accelerator wheels and amplifier wheels to score into the amplifier
    acceleratorWheel.set(-1); // sets the accelerator wheels to run backwatds
    amplifierWheel.set(1); // sets the amp wheel to run forwards
  }

  public void intakeNote() { // runs the accelerator wheels and amplifier wheels to intake a note
    acceleratorWheel.set(0.75); // sets the accelerator wheels to run at 75%
    amplifierWheel.set(ShooterConstants.kIntakeSpeed); // sets the amplifier wheels to run at full speed
  }

  /**
   * sets the amp wheel speed
   * 
   * @param ampSpeed speed to set the amp motor to
   */
  public void setAmpWheel(double ampSpeed) {
    amplifierWheel.set(ampSpeed); // sets the speed of our amplifier wheel to any speed we want
  }

  /**
   * 
   * @param targetRPM
   */
  public void setShooterVelocity(double targetRPM) {

    shooterMotorMaster.setControl(shooterTV.withVelocity(targetRPM / 60).withFeedForward(20)); // sets our shooter motor
                                                                                               // to a set speed with a
                                                                                               // feed forward
    shooterMotorSlave.setControl(shooterTV.withVelocity(targetRPM / 60).withFeedForward(20));

  }

  public double getShooterVelocity() {
    double shooterVelocity = shooterMotorMaster.getVelocity().getValue() * 60; // finds the velocity our shooter is
                                                                               // currently running at
    return shooterVelocity;
  }

  public void stopShooter() { // stops shooter motor
    shooterMotorMaster.stopMotor();
    shooterMotorSlave.stopMotor();
  }

  public void stopAcceleratorWheel() { // stops accelerator wheel
    acceleratorWheel.stopMotor();

  }

  public void stopAmplifierWheel() { // stops amplifier wheel
    amplifierWheel.stopMotor();
  }

  /**
   * Method for getting the status of the Shot Beam Break
   * 
   * @return true when a note has broken the beam
   */
  public boolean getShotBeambreak() { // returns true when the shooter beam break is broken
    return !beamBreakShot.get();

  }

  public boolean getAmpBeambreak() { // returns true when the amp shot's beam break is broken
    return !beamBreakAmp.get();

  }

  public boolean getMiddleBackBeambreak() { // returns true when the middle back beam break is broken
    return !beamBreakMiddleBack.get();
  }

  public boolean getMiddleFrontBeambreak() { // returns true when the middle front beam break is broken
    return !beamBreakMiddleFront.get();
  }

  /**
   * Method for getting the status of the front Time of Flight Sensor
   * 
   * @return true if there is a note in front of the sensor
   */
  public boolean getFrontTOFSensor() {
    return frontTOF.getRange() < 100;
  }

  public boolean hasAutoNote() {
    return getAmpBeambreak() ||
        getMiddleBackBeambreak() ||
        getMiddleFrontBeambreak() ||
        getShotBeambreak();
  }

  public void SetIntakeWheelsBack() {
    amplifierWheel.set(-0.5); // runs amp wheels backwards
    acceleratorWheel.set(-0.5); // runs accelerator wheels backwards
  }

  public void teleopTorqueCurrent() {
    shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40; // sets a maximum current for motors during teleop
    shooterMotorMaster.getConfigurator().apply(shooterConfig); // assigns current to motor master
    shooterMotorSlave.getConfigurator().apply(shooterConfig); // assigns current to motor slave
  }

  public void autoTorqueCurrent() {
    shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 65; // sets a maximum current for motors during auto
    shooterMotorMaster.getConfigurator().apply(shooterConfig); // assigns current to motor master
    shooterMotorSlave.getConfigurator().apply(shooterConfig); // assigns current to motor slave
  }

}