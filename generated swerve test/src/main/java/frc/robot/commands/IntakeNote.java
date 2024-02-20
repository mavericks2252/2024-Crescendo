// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  Shooter shooter;
  Intake intake;
  ShooterRotationSubsystem shooterRotationSubsystem;
  double speed;

  public IntakeNote(Intake intake, ShooterRotationSubsystem shooterRotationSubsystem, Shooter shooter) {
    this.intake = intake;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooterRotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooterRotationSubsystem.setIntakeMode();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (shooterRotationSubsystem.isAngleOnTarget()) {
      intake.setIntakeSpeed();
      shooter.setAmpWheel(1);

      if (shooter.getMiddleBackBeambreak()) { // if middle beam break is broken
        shooter.acceleratorWheelOutput(0.4);

      } else {
        shooter.acceleratorWheelOutput(1); // sets to full speed when no note is in middle beam break
      }
    }

    else {
      intake.stopIntake();
      shooter.stopAmplifierWheel();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.stopIntake();
    shooter.stopAcceleratorWheel();
    shooter.stopAmplifierWheel();
    if (shooter.getMiddleFrontBeambreak() || shooter.getMiddleBackBeambreak())
      shooterRotationSubsystem.setSpeakerTracking();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return shooter.getMiddleFrontBeambreak();

  }
}
