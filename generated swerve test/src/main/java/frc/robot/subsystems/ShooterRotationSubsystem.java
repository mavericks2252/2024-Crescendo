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
    shooterAngleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterAngleConfig.CurrentLimits.StatorCurrentLimit = 15;
    FeedbackConfigs fdb = shooterAngleConfig.Feedback;
    fdb.SensorToMechanismRatio = ShooterConstants.kShooterGearBoxRatio;

    throughBoreEncoder = new DutyCycleEncoder(PortConstants.kThroughBoreEncoder);
    throughBoreEncoder.setPositionOffset(0.2301);

    shooterAngleMotor = new TalonFX(PortConstants.kShooterAngleMotorPort);
    shooterAngleMotor.getConfigurator().apply(shooterAngleConfig);
    shooterAngleMotor.setInverted(false);
    shooterAngleMotor.setNeutralMode(NeutralModeValue.Brake);

    SmartDashboard.putNumber("test shooter angle", 90);

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
      setShooterAngle(photon.getTargetAngle()); // sets shooter to track the
      // speaker
    }

    else if (intakeMode) {

      setShooterIntakeAngle(); // sets shooter to position to intake

    }

    else if (ampMode) {
      if (photon.getAmpDistance() < 2)
        setShooterAmpAngle(); // sets shooter to point for shooting into amp if within 1 meter of it
      else
        setShooterAngle(100); // sets shooter to flat if not within 1 meter
    }

  }

  public static double getThroughBoreEncoder() {
    return throughBoreEncoder.get(); // gets the position of the shooter's rotation
  }

  public double getAngleMotorError() {
    return shooterAngleMotor.getClosedLoopError().refresh().getValueAsDouble(); // gives the error between the motor and
                                                                                // where it should be
  }

  public boolean isAngleOnTarget() {
    double tolerance;
    double target = 0;
    if (speakerTracking()) { // if we are tracking speaker
      tolerance = degreesToMotorRevs(0.5); // sets the allowed error to half a degree
    } else { // if we are tracking anything else
      tolerance = degreesToMotorRevs(2); // sets the allowed error to 2 degrees
    }

    if (speakerAngleTracking) {
      target = photon.getTargetAngle();
    } else if (intakeMode) {
      target = degreesToMotorRevs(ShooterConstants.kIntakeAngle);
    } else if (ampMode) {
      target = degreesToMotorRevs(ShooterConstants.kAmpAngle);
    }

    double error = Math.abs(getAngleMotorPos() - target);

    return error < tolerance;
    // return getAngleMotorError() < tolerance; // tells us whether the motor is
    // within our tolerence
  }

  public void setShooterIntakeAngle() {
    double motorTarget = degreesToMotorRevs(ShooterConstants.kIntakeAngle); // sets motorTarget equal to the posision
                                                                            // for intaking

    shooterAngleMotor.setControl(m_PositionDutyCycle.withPosition(motorTarget)); // sets our shooter to the motorTarget
                                                                                 // position
  }

  public void setShooterAmpAngle() {
    double motorTarget = degreesToMotorRevs(ShooterConstants.kAmpAngle); // sets motorTarget equal to the posision for
                                                                         // scoring in the amp

    shooterAngleMotor.setControl(m_PositionDutyCycle.withPosition(motorTarget)); // sets our shooter to the motorTarget
                                                                                 // position
  }

  public void setShooterAngle(double targetDegrees) {
    double targetRevs = degreesToMotorRevs(targetDegrees); // sets targetRevs equal to a position we choose
    shooterAngleMotor.setControl(m_PositionDutyCycle.withPosition(targetRevs)); // sets our shooter to the position we
                                                                                // chose
  }

  public void setSpeakerTracking() { // sets the shooter to our mode that tracks the speaker and turns off all other
                                     // modes
    speakerAngleTracking = true;
    intakeMode = false;
    ampMode = false;
  }

  public void setIntakeMode() { // sets the shooter to our mode that tracks the intake and turns off all other
                                // modes
    intakeMode = true;
    speakerAngleTracking = false;
    ampMode = false;
  }

  public void setAmpMode() { // sets the shooter to our mode that tracks the amp and turns off all other
                             // modes
    ampMode = true;
    speakerAngleTracking = false;
    intakeMode = false;
  }

  public boolean speakerTracking() {
    return speakerAngleTracking; // returns true if we are tracking the speaker
  }

  public double getAngleMotorPos() {
    return shooterAngleMotor.getPosition().getValue();
  }

  public double motorRevsToDegrees(double revs) {// converts motor revolutions to degrees
    return revs * 360;
  }

  public double degreesToMotorRevs(double degrees) {// converts degrees to motor revolutions
    return degrees / 360;
  }

  public static void setShooterAngleMotorSensorPos() {
    int loops = 50;
    double encoderReadings = 0;
    for (int i = 0; i < loops; i++) {
      encoderReadings += getThroughBoreEncoder(); // averages out the reding from our encoder after 50 loops of it
    }
    double averageEncoderReadings = encoderReadings / loops; // divides the final number by the number of loops to get
                                                             // the average

    SmartDashboard.putNumber("encoder Readings", averageEncoderReadings);
    shooterAngleMotor.setPosition(averageEncoderReadings + 0.001214); // adds an offset to the position and makes the
                                                                      // motor go to it

  }

  public void setRobotEnablePos() {

    shooterAngleMotor.setControl(m_PositionDutyCycle.withPosition(getAngleMotorPos())); // sets the pose of the shooter
                                                                                        // when the robot is first
                                                                                        // enabled
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