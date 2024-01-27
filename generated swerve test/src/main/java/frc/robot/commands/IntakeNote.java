// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  Intake intake;
  double speed;
  double motorMasterSpeed;
  double motorSlaveSpeed;
  public IntakeNote(Intake intake, double motorMasterSpeed, double motorSlaveSpeed) {
    this.intake = intake;
    this.motorMasterSpeed = motorMasterSpeed;
    this.motorSlaveSpeed = motorSlaveSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.deployIntake();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.setPercentOutput(motorMasterSpeed, motorSlaveSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.retractIntake();
    intake.stopIntake();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // return intake.getBeamBreak();
   return false;
    
  }
}
