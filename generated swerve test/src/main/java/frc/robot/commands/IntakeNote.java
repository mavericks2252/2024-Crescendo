// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  Shooter shooter;
  Intake intake;
  ShooterRotationSubsystem shooterRotationSubsystem;
  double speed;
  LEDSubsystem ledSubsystem;

  public IntakeNote(Intake intake, ShooterRotationSubsystem shooterRotationSubsystem, Shooter shooter,
      LEDSubsystem ledSubsystem) {
    this.intake = intake;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.shooter = shooter;
    this.ledSubsystem = ledSubsystem;

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

    RobotContainer.m_driver_controler.getHID().setRumble(RumbleType.kBothRumble, 1);

    if (shooterRotationSubsystem.isAngleOnTarget()) {
      intake.setIntakeSpeed(); // runs the front intake wheels
      shooter.setAmpWheel(1); // runs the amplifier wheels at 100%
    }
    if (shooter.getMiddleBackBeambreak()) { // if middle beam break is broken
      shooter.acceleratorWheelOutput(0.4); // runs the accelerator wheels at 40%

    } else {
      shooter.acceleratorWheelOutput(1); // sets to full speed when no note is in middle beam break
    }
    /*
     * }
     * 
     * else {
     * intake.stopIntake();
     * shooter.stopAmplifierWheel();
     * }
     */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.stopIntake(); // stops the front intake wheels
    shooter.stopAcceleratorWheel(); // stops the accelerator wheels
    shooter.stopAmplifierWheel(); // stops the amplifier wheels
    if (shooter.getMiddleFrontBeambreak() || shooter.getMiddleBackBeambreak()) // if the middle front or middle back
                                                                               // beam brakes
      shooterRotationSubsystem.setSpeakerTracking(); // set the shooter to speaker tracking
    RobotContainer.m_driver_controler.getHID().setRumble(RumbleType.kBothRumble, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return shooter.getMiddleFrontBeambreak(); // will stop when middle front beam
    // brake is broken

  }
}
