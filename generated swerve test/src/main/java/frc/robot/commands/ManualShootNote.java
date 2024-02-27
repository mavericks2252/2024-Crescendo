// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;

public class ManualShootNote extends Command {
  /** Creates a new ShootNote. */
  Shooter shooter;
  ShooterRotationSubsystem shooterRotationSubsystem;

  double targetRPM;

  public ManualShootNote(Shooter shooter, ShooterRotationSubsystem shooterRotationSubsystem) {
    this.shooter = shooter;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterRotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterRotationSubsystem.setShooterAngle(112.25); // sets the shooter to track the speaker
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetRPM = 3500; // sets the shooter to a desired rpm
    // shooterRotationSubsystem.setShooterAngle(photon.getTargetAngle()); // sets
    // the angle that the shooter needs to
    // target

    if (targetRPM - shooter.getShooterVelocity() < 75) { // if the shooter is within 75 rpm of the desired speed
      shooter.acceleratorWheelOutput(0.95); // sets the accelerator wheel to full speed
    }

    shooter.setShooterVelocity(targetRPM); // sets the speed of the shooter when the command starts

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stopShooter(); // stops the front shooter modes
    shooter.stopAcceleratorWheel(); // stops the accelerator wheels
    shooter.stopAmplifierWheel();
    shooterRotationSubsystem.setIntakeMode(); // sets the shooter into intake mode

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
