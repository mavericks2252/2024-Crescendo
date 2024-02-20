// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class SetAmpMode extends Command {

  double loops;
  Shooter shooter;
  Intake intake;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;

  /** Creates a new AmpPreStage. */
  public SetAmpMode(Intake intake, ShooterRotationSubsystem shooterRotationSubsystem, Shooter shooter,
      VisionPhotonSubsystem photon) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.shooter = shooter;
    this.photon = photon;

    loops = 0;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    addRequirements(shooterRotationSubsystem, shooter);
    shooterRotationSubsystem.setAmpMode();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.ampScore();

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
    if (!shooter.getMiddleFrontBeambreak()) {
      if (loops < 1) {
        loops++;
        return false;
      } else
        return true;
    } else
      return false;
  }
}
