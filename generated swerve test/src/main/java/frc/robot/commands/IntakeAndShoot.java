// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class IntakeAndShoot extends Command {
  Intake intake;
  Shooter shooter;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;
  double targetRPM, error;

  /** Creates a new IntakeAndShoot. */
  public IntakeAndShoot(Intake intake, Shooter shooter, ShooterRotationSubsystem shooterRotationSubsystem,
      VisionPhotonSubsystem photon) {
    this.intake = intake;
    this.shooter = shooter;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.photon = photon;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake, shooterRotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooterRotationSubsystem.setSpeakerTracking();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetRPM = 2200;
    error = targetRPM - shooter.getShooterVelocity();
    shooter.setShooterVelocity(targetRPM);
    if (error < 100)
      shooter.acceleratorWheelOutput(0.95);

    if (!shooter.getMiddleFrontBeambreak() && !shooter.getMiddleBackBeambreak()) {
      intake.setIntakeSpeed();
      shooter.setAmpWheel(1);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    shooter.stopAcceleratorWheel();
    shooter.stopAmplifierWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
