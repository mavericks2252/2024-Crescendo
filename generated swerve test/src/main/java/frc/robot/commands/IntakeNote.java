// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  Shooter shooter;
  Intake intake;
  ShooterRotationSubsystem shooterRotationSubsystem;
  double speed;
  double motorMasterSpeed;
  double acceleratorWheelSpeed;
  double centeringMotorSpeed;
  
  public IntakeNote(Intake intake, ShooterRotationSubsystem shooterRotationSubsystem, double motorMasterSpeed, Shooter shooter) {
    this.intake = intake;
    this.shooterRotationSubsystem = shooterRotationSubsystem;    
    this.motorMasterSpeed = motorMasterSpeed;
    this.shooter = shooter;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooterRotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.setPercentOutput(motorMasterSpeed, centeringMotorSpeed);
    shooter.intakeNote(ShooterConstants.kIntakeSpeed);
    

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
   //return intake.getBeamBreak();
   return false;
   
    
  }
}
