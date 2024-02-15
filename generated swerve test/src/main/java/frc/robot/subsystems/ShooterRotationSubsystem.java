// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;

public class ShooterRotationSubsystem extends SubsystemBase {
  static DutyCycleEncoder throughBoreEncoder;
  ProfiledPIDController autoAnglePIDController;
  static TalonFX shooterAngleMotor;
  // VisionSubsystem vision;
  VisionPhotonSubsystem photon;
  boolean speakerAngleTracking = false;

  TrapezoidProfile.Constraints angle_PIDConstraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate,
      TunerConstants.kMaxAngularAcceleration);

  /** Creates a new ShooterRotationSubsystem. */
  public ShooterRotationSubsystem(VisionPhotonSubsystem photon) {
    // this.vision = vision;
    this.photon = photon;
    /*
     * TalonFXConfiguration shooterAngleConfig = new TalonFXConfiguration();
     * shooterAngleConfig.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
     * shooterAngleConfig.Slot2.kP = 0.1;
     * shooterAngleConfig.Slot2.kI = 0;
     * shooterAngleConfig.Slot2.kD = 0;
     * shooterAngleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
     * shooterAngleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
     * shooterAngleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
     * degreesToMotorRevs(
     * ShooterConstants.kForwardSoftLimit);
     * shooterAngleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
     * degreesToMotorRevs(
     * ShooterConstants.kReverseSoftLimit);
     * FeedbackConfigs fdb = shooterAngleConfig.Feedback;
     * fdb.SensorToMechanismRatio = ShooterConstants.kShooterGearBoxRatio;
     */

    throughBoreEncoder = new DutyCycleEncoder(PortConstants.kThroughBoreEncoder);

    shooterAngleMotor = new TalonFX(PortConstants.kShooterAngleMotorPort);
    // shooterAngleMotor.getConfigurator().apply(shooterAngleConfig);
    shooterAngleMotor.setInverted(false);
    shooterAngleMotor.setNeutralMode(NeutralModeValue.Brake);

    throughBoreEncoder.setPositionOffset(0.5635);

    autoAnglePIDController = new ProfiledPIDController(2, 0.25, 0, angle_PIDConstraints, 0.01);
    autoAnglePIDController.enableContinuousInput(0, 360);
    // autoAnglePIDController.setIZone(0);

    new Thread(() -> {
      try {
        Thread.sleep(2500);

        setShooterAngleMotorPos();

      } catch (Exception e) {

      }
    }).start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("poition Offset", throughBoreEncoder.getPositionOffset());
    SmartDashboard.putBoolean("is Speaker tracking", speakerTracking());

    SmartDashboard.putNumber("ThroughBore Encoder", getThroughBoreEncoder());

    SmartDashboard.putNumber("Motor Position Degrees", getAngleMotorPos());

    /*
     * if(speakerAngleTracking){
     * shooterAngleMotor.set(getAutoAngleOutput(photon.getTargetAngle()));
     * }
     */
  }

  public static double getThroughBoreEncoder() {
    return throughBoreEncoder.get();
  }

  public double getAutoAngleOutput(double target) {
    double currentPos = getThroughBoreEncoder();
    double targetPos = degreesToMotorRevs(target);
    SmartDashboard.putNumber("Angle Target", targetPos);
    double output = autoAnglePIDController.calculate(currentPos, targetPos);

    SmartDashboard.putNumber("Auto Angle output", output);
    return output;
  }

  public void setShooterIntakeAngle() {
    speakerAngleTracking = false;
    shooterAngleMotor.set(getAutoAngleOutput(ShooterConstants.kIntakeAngle));
  }

  public void setShooterAmpAngle() {
    speakerAngleTracking = false;
    shooterAngleMotor.set(getAutoAngleOutput(ShooterConstants.kAmpAngle));
  }

  public void setSpeakerTracking() {
    speakerAngleTracking = true;
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

  public static void setShooterAngleMotorPos() {
    shooterAngleMotor.setPosition(getThroughBoreEncoder());
  }

}
