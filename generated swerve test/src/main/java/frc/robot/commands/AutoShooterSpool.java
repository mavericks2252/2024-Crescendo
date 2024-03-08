// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class AutoShooterSpool extends Command {
  Shooter shooter;
  VisionPhotonSubsystem photon;
  ShooterRotationSubsystem shooterRotationSubsystem;
  double targetRPM;

  /** Creates a new AutoShooterSpool. */
  public AutoShooterSpool(Shooter shooter, VisionPhotonSubsystem photon,
      ShooterRotationSubsystem shooterRotationSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.photon = photon;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (photon.getSpeakerDistance() < 4)
      targetRPM = 426.8 * photon.getSpeakerDistance() + 1792.8;
    else
      targetRPM = -600 * photon.getSpeakerDistance() + 6500;
    if (targetRPM > 3500)
      targetRPM = 3500;
    if ((shooter.getMiddleFrontBeambreak() || shooter.getMiddleBackBeambreak()) && photon.getSpeakerDistance() < 7.5
        && shooterRotationSubsystem.getSpeakerTracking()) {
      shooter.setShooterVelocity(targetRPM);
    } else
      shooter.stopShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.stopAcceleratorWheel();
    shooter.stopAmplifierWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
