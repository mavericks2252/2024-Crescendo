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

  Boolean hasNote;
  boolean seesNote;
  public Boolean isIntaking;

  public Spark blinkin;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(Shooter shooter, Intake intake, VisionPhotonSubsystem photon,
      ShooterRotationSubsystem shooterRotationSubsystem) {
    this.shooter = shooter;
    this.intake = intake;
    this.photon = photon;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    blinkin = new Spark(9);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intaking, sees note, score mode & has note
    hasNote = (shooter.getMiddleFrontBeambreak() &&
        shooterRotationSubsystem.getSpeakerTracking());
    seesNote = (photon.noteCam.getLatestResult().hasTargets());

    /*
     * if (isIntaking) {
     * blinkin.set(BlinkinConstants.kOrange);
     * }
     * 
     * else if (hasNote) {
     * blinkin.set(BlinkinConstants.kGreen);
     * }
     * else if (seesNote) {
     * blinkin.set(0.05);
     * }
     * 
     * else
     * teamLEDColor();
     */
  }

  public void teamLEDColor() {
    if (DriverStation.getAlliance().isPresent()) { // if we are getting an alliance color
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) // if its blue
        blinkin.set(BlinkinConstants.kBlue); // make the lights blue
      else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)// if its red
        blinkin.set(BlinkinConstants.kRed); // make the lights red
    }
  }

}
