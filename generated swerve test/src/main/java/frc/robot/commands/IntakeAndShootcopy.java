// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
  double rpm1, rpm2, delayTime, shooterAngle1, shooterAngle2, targetAngle;
  Timer delayTimer = new Timer();

  /** Creates a new IntakeAndShoot. */
  public IntakeAndShootcopy(Intake intake, Shooter shooter, ShooterRotationSubsystem shooterRotationSubsystem,
      VisionPhotonSubsystem photon) {
    delayTime = 2;
    rpm1 = 2000;
    rpm2 = 2500;
    shooterAngle1 = 131;
    shooterAngle2 = 125;
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

    delayTimer.restart();

    shooterRotationSubsystem.setManualShoot();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (delayTimer.get() < delayTime) {
      targetRPM = rpm1;
      targetAngle = shooterAngle1;
    } else {
      targetRPM = rpm2;
      targetAngle = shooterAngle2;
    }
    error = targetRPM - shooter.getShooterVelocity();
    shooterRotationSubsystem.setShooterAngle(targetAngle);
    shooter.setShooterVelocity(targetRPM);
    if (error < 125)
      shooter.acceleratorWheelOutput(0.95);

    if (!shooter.getMiddleFrontBeambreak() && !shooter.getMiddleBackBeambreak() && !shooter.getShotBeambreak()) {
      intake.setIntakeSpeed();
      shooter.setAmpWheel(1);
    }

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
    return false;
  }
}
