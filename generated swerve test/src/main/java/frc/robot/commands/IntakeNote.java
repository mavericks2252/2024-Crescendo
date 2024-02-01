// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  Shooter shooter;
  Intake intake;
  double speed;
  double motorMasterSpeed;
  double acceleratorWheelSpeed;
  
  public IntakeNote(Intake intake, double motorMasterSpeed, double acceleratorWheelSpeed, Shooter shooter) {
    this.intake = intake;
    this.motorMasterSpeed = motorMasterSpeed;
    this.shooter = shooter;
   this.acceleratorWheelSpeed = acceleratorWheelSpeed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.setPercentOutput(motorMasterSpeed);
    shooter.acceleratorWheelOutput(acceleratorWheelSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.stopIntake();
    shooter.stopAcceleratorWheel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return intake.getBeamBreak();
   
    
  }
}
