// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class IntakeAndShootcopy extends Command {
  Intake intake;
  Shooter shooter;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;
  double targetRPM, error;
  double shooterAngle1, shooterAngle2, targetAngle;

  /** Creates a new IntakeAndShoot. */
  public IntakeAndShootcopy(Intake intake, Shooter shooter, ShooterRotationSubsystem shooterRotationSubsystem,
      VisionPhotonSubsystem photon) {

    targetRPM = 3500;
    targetAngle = 111.75;
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

    shooterRotationSubsystem.setShooterAngle(targetAngle);
    shooterRotationSubsystem.setManualShoot();
    shooter.setShooterVelocity(targetRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    error = targetRPM - shooter.getShooterVelocity();

    if (error < 100)
      shooter.acceleratorWheelOutput(0.95);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterRotationSubsystem.setSpeakerTracking();
    intake.stopIntake();
    shooter.stopAcceleratorWheel();
    shooter.stopAmplifierWheel();
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!shooter.getMiddleBackBeambreak() && !shooter.getMiddleFrontBeambreak() && !shooter.getShotBeambreak())
      return true;
    else
      return false;
  }
}
