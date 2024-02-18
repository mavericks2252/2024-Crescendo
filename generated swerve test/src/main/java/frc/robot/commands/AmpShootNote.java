// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class AmpShootNote extends Command {
  /** Creates a new AmpShootNote. */

  Shooter shooter;
  Intake intake;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;

  public AmpShootNote(Intake intake, ShooterRotationSubsystem shooterRotationSubsystem, Shooter shooter,
      VisionPhotonSubsystem photon) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.shooter = shooter;
    this.photon = photon;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    addRequirements(shooterRotationSubsystem);
    shooterRotationSubsystem.setShooterAmpAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (shooterRotationSubsystem.getAngleMotorError() < 0.0139) {
      shooter.ampScore();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAcceleratorWheel();
    shooter.stopAmplifierWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
