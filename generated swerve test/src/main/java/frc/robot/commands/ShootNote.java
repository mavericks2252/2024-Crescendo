// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  /** Creates a new ShootNote. */
  Shooter shooter;
  double topMotorRpm;
  double bottomMotorRpm;
  double topMotorSpeed;
  double bottomMotorSpeed;
  double targetRPM;
  public ShootNote(Shooter shooter, double bottomMotorRpm, double topMotorRpm, double bottomMotorSpeed, double topMotorSpeed, double targetRPM) {
    this.shooter = shooter;
    //this.acceleratorWheelSpeed = acceleratorWheelSpeed;
    this.topMotorRpm = topMotorRpm;
    this.bottomMotorRpm = bottomMotorRpm;
    this.topMotorSpeed = topMotorSpeed;
    this.bottomMotorSpeed = bottomMotorSpeed;
    this.targetRPM = targetRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (ShooterConstants.kShooterMotorMasterSpeed - shooter.getShooterVelocity() < 200){
      shooter.acceleratorWheelOutput(ShooterConstants.kacceleratorWheelSpeed);
    }

    //shooter.setPercentOutput(topMotorSpeed, bottomMotorSpeed);
    shooter.setShooterVelocity(targetRPM);
    shooter.acceleratorWheelOutput(1);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stopShooter();
    //  shooter.setShooterVelocity(1500);
    shooter.stopAcceleratorWheel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
