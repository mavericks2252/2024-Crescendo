// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  /** Creates a new ShootNote. */
  Shooter shooter;
  double topMotorSpeed;
  double bottomMotorSpeed;
  double feedMotorSpeed;
  public ShootNote(Shooter shooter, double topMotorSpeed, double bottomMotorSpeed, double feedMotorSpeed) {
    this.shooter = shooter;
    this.topMotorSpeed = topMotorSpeed;
    this.bottomMotorSpeed = bottomMotorSpeed;
    this.feedMotorSpeed = feedMotorSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setPercentOutput(topMotorSpeed, bottomMotorSpeed);
    shooter.feedMotorOutput(feedMotorSpeed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stopShooter();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
