// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class ShootNote extends Command {
  /** Creates a new ShootNote. */
  Shooter shooter;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;

  double targetRPM;

  public ShootNote(Shooter shooter, ShooterRotationSubsystem shooterRotationSubsystem, VisionPhotonSubsystem photon) {
    this.shooter = shooter;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.photon = photon;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterRotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterRotationSubsystem.setSpeakerTracking();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetRPM = photon.getTargetRPM();
    shooterRotationSubsystem.setShooterAngle(photon.getTargetAngle());

    if (targetRPM - shooter.getShooterVelocity() < 75) {
      shooter.acceleratorWheelOutput(1);
    }

    shooter.setShooterVelocity(targetRPM);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stopShooter();
    shooter.stopAcceleratorWheel();
    shooterRotationSubsystem.setIntakeMode();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
