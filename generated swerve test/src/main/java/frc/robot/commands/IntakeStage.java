// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeStage extends Command {
  Shooter shooter;
  Intake intake;

  /** Creates a new IntakeSpool. */
  public IntakeStage(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;

    addRequirements(shooter, intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getMiddleFrontBeambreak()) { // if the middle front beambreak is broken
      shooter.setAmpWheel(-0.25); // set amp wheels backwards
      shooter.acceleratorWheelOutput(-0.25); // and set accelerator wheels backwards
    } else { // once middle front is no longer broken
      shooter.stopAmplifierWheel(); // stop amplifier wheels
      shooter.stopAcceleratorWheel(); // stop accelerator wheels
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAmplifierWheel(); // stop amplfier wheels
    shooter.stopAcceleratorWheel(); // stop accelerator wheels
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return !shooter.getMiddleFrontBeambreak(); // stops wel middle front beambreak is broken
  }
}
