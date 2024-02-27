// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class SetAmpMode extends Command {

  double loops;
  Shooter shooter;
  Intake intake;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;
  Boolean hasBeenBroken;

  /** Creates a new AmpPreStage. */
  public SetAmpMode(Intake intake, ShooterRotationSubsystem shooterRotationSubsystem, Shooter shooter,
      VisionPhotonSubsystem photon) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.shooter = shooter;
    this.photon = photon;

    hasBeenBroken = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    addRequirements(shooterRotationSubsystem, shooter);
    shooterRotationSubsystem.setAmpMode(); // sets the shooter to amp mode
    loops = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * if (shooter.getMiddleFrontBeambreak()) {
     * shooter.ampScore(); // runs the amp and accelerator wheels as to score into
     * the amp
     * } else {
     * shooter.acceleratorWheelOutput(0.2);
     * }
     */
    if (shooter.getMiddleFrontBeambreak()) {
      hasBeenBroken = true;
    }

    if (!shooter.getMiddleFrontBeambreak() && !hasBeenBroken) {
      shooter.acceleratorWheelOutput(0.2);
    } else {
      shooter.acceleratorWheelOutput(-0.5);
      shooter.setAmpWheel(0.5);
    }
    if (shooter.getAmpBeambreak())
      loops++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stopAcceleratorWheel(); // stops the accelerator wheels
    shooter.stopAmplifierWheel(); // stops the amplifier wheels

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (loops < 4) { // and loops is less than 1
      return false; // and dont stop
    } else // if loops is more than 1
      return true; // stop

  }
}
