// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class ShootNote extends Command {
  /** Creates a new ShootNote. */
  Shooter shooter;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;

  double targetRPM;

  public ShootNote(Shooter shooter, ShooterRotationSubsystem shooterRotationSubsystem, VisionPhotonSubsystem photon,
      double targetRPM) {
    this.shooter = shooter;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.photon = photon;
    this.targetRPM = targetRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterRotationSubsystem);
    SmartDashboard.putNumber("test shoot RPM", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterRotationSubsystem.setSpeakerTracking();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // shooterRotationSubsystem.setShooterAngle(photon.getTargetAngle());

    if (3500 - shooter.getShooterVelocity() < 75) {
      shooter.acceleratorWheelOutput(1);
    }

    // shooter.setShooterVelocity(targetRPM);
    shooter.setShooterVelocity(3500);

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
