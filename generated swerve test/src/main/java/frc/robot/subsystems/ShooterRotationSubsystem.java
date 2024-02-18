// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;

public class ShooterRotationSubsystem extends SubsystemBase {
  static DutyCycleEncoder throughBoreEncoder;
  static TalonFX shooterAngleMotor;
  private final PositionDutyCycle m_PositionDutyCycle = new PositionDutyCycle(0, 0, false, 0.03, 2, false, false,
      false);
  // VisionSubsystem vision;
  VisionPhotonSubsystem photon;
  boolean speakerAngleTracking = false;
  boolean intakeMode = false;
  boolean ampMode = false;

  TrapezoidProfile.Constraints angle_PIDConstraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate,
      TunerConstants.kMaxAngularAcceleration);

  /** Creates a new ShooterRotationSubsystem. */
  public ShooterRotationSubsystem(VisionPhotonSubsystem photon) {
    // this.vision = vision;
    this.photon = photon;

    TalonFXConfiguration shooterAngleConfig = new TalonFXConfiguration();
    shooterAngleConfig.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
    shooterAngleConfig.Slot2.kP = 30;
    shooterAngleConfig.Slot2.kI = 0;
    shooterAngleConfig.Slot2.kD = 0;
    shooterAngleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    shooterAngleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    shooterAngleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRevs(
        ShooterConstants.kForwardSoftLimit);
    shooterAngleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRevs(
        ShooterConstants.kReverseSoftLimit);
    FeedbackConfigs fdb = shooterAngleConfig.Feedback;
    fdb.SensorToMechanismRatio = ShooterConstants.kShooterGearBoxRatio;

    throughBoreEncoder = new DutyCycleEncoder(PortConstants.kThroughBoreEncoder);
    throughBoreEncoder.setPositionOffset(0.2301);

    shooterAngleMotor = new TalonFX(PortConstants.kShooterAngleMotorPort);
    shooterAngleMotor.getConfigurator().apply(shooterAngleConfig);
    shooterAngleMotor.setInverted(false);
    shooterAngleMotor.setNeutralMode(NeutralModeValue.Brake);

    // SmartDashboard.putNumber("test shooter angle", 90);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ThroughBore Encoder", getThroughBoreEncoder());
    SmartDashboard.putNumber("ThroughBoreEncoderDegrees", motorRevsToDegrees(getThroughBoreEncoder()));
    SmartDashboard.putNumber("Angle Motor Position", getAngleMotorPos());
    SmartDashboard.putNumber("Angle Motor Position Degrees", getAngleMotorPos() * 360);
    SmartDashboard.putNumber("Rotation position Error", getAngleMotorError());

    if (speakerAngleTracking) {

      // setShooterAngle(SmartDashboard.getNumber("test shooter angle", 90));
      setShooterAngle(photon.getTargetAngle());
    }

    else if (intakeMode) {

      setShooterIntakeAngle();

    }

    else if (ampMode) {
      setShooterAmpAngle();
    }

  }

  public static double getThroughBoreEncoder() {
    return throughBoreEncoder.get();
  }

  public double getAngleMotorError() {
    return shooterAngleMotor.getClosedLoopError().refresh().getValueAsDouble();
  }

  public boolean isAngleOnTarget() {
    double tolerance;
    if (speakerTracking()) {
      tolerance = degreesToMotorRevs(0.5);
    } else {
      tolerance = degreesToMotorRevs(2);
    }

    return getAngleMotorError() < tolerance;
  }

  public void setShooterIntakeAngle() {
    double motorTarget = degreesToMotorRevs(ShooterConstants.kIntakeAngle);

    shooterAngleMotor.setControl(m_PositionDutyCycle.withPosition(motorTarget));
  }

  public void setShooterAmpAngle() {
    double motorTarget = degreesToMotorRevs(ShooterConstants.kAmpAngle);

    shooterAngleMotor.setControl(m_PositionDutyCycle.withPosition(motorTarget));
  }

  public void setShooterAngle(double targetDegrees) {
    double targetRevs = degreesToMotorRevs(targetDegrees);
    shooterAngleMotor.setControl(m_PositionDutyCycle.withPosition(targetRevs));
  }

  public void setSpeakerTracking() {
    speakerAngleTracking = true;
    intakeMode = false;
    ampMode = false;
  }

  public void setIntakeMode() {
    intakeMode = true;
    speakerAngleTracking = false;
    ampMode = false;
  }

  public void setAmpMode() {
    ampMode = true;
    speakerAngleTracking = false;
    intakeMode = false;
  }

  public boolean speakerTracking() {
    return speakerAngleTracking;
  }

  public double getAngleMotorPos() {
    return shooterAngleMotor.getPosition().getValue();
  }

  public double motorRevsToDegrees(double revs) {
    return revs * 360;
  }

  public double degreesToMotorRevs(double degrees) {
    return degrees / 360;
  }

  public static void setShooterAngleMotorSensorPos() {
    int loops = 50;
    double encoderReadings = 0;
    for (int i = 0; i < loops; i++) {
      encoderReadings += getThroughBoreEncoder();
    }
    double averageEncoderReadings = encoderReadings / loops;

    SmartDashboard.putNumber("encoder Readings", averageEncoderReadings);
    shooterAngleMotor.setPosition(averageEncoderReadings + 0.001214);

  }

  public void setRobotEnablePos() {

    shooterAngleMotor.setControl(m_PositionDutyCycle.withPosition(getAngleMotorPos()));
  }

}

/*
 * public double getAutoAngleOutput(double target) {
 * double currentPos = getThroughBoreEncoder();
 * double targetPos = degreesToMotorRevs(target);
 * SmartDashboard.putNumber("Angle Target", targetPos);
 * double output = autoAnglePIDController.calculate(currentPos, targetPos);
 * 
 * SmartDashboard.putNumber("Auto Angle output", output);
 * return output;
 * }
 * 
 */