// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.BlinkinConstants;

public class LEDSubsystem extends SubsystemBase {
  Shooter shooter;
  CommandSwerveDrivetrain drivetrain;
  Intake intake;
  VisionPhotonSubsystem photon;
  ShooterRotationSubsystem shooterRotationSubsystem;

  Spark blinkin;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(Shooter shooter, Intake intake, VisionPhotonSubsystem photon,
      ShooterRotationSubsystem shooterRotationSubsystem) {
    this.shooter = shooter;
    this.intake = intake;
    this.photon = photon;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    blinkin = new Spark(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intaking, sees note, score mode & has note

    if (shooter.getMiddleFrontBeambreak() && shooterRotationSubsystem.getSpeakerTracking())
      blinkin.set(BlinkinConstants.kGreen);

    else if (intake.intakeMotor.get() < 0)
      blinkin.set(BlinkinConstants.kOrange);

    else if (photon.noteCam.getLatestResult().hasTargets())
      blinkin.set(0.05);

    else
      teamLEDColor();

  }

  public void teamLEDColor() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        blinkin.set(BlinkinConstants.kBlue);
      else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        blinkin.set(BlinkinConstants.kRed);
    }
  }

}
