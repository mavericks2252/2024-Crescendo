// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
//import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class AutoNoteIntake extends Command {
  /** Creates a new NoteAim. */
  VisionPhotonSubsystem photon;
  Intake intake;
  CommandSwerveDrivetrain drivetrain;
  // AutoAimSubsystem autoAimSubsystem;

  private final SwerveRequest.RobotCentric autoNote = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoNoteIntake(VisionPhotonSubsystem photon,
      Intake intake,
      CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photon = photon;
    this.intake = intake;
    this.drivetrain = drivetrain;
    // this.autoAimSubsystem = autoAimSubsystem;

    addRequirements(intake, drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    photon.setPhotonPipeline(VisionConstants.kNotePipeline);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (photon.camera.getLatestResult().hasTargets()) {
      intake.setIntakeSpeed();

      drivetrain.applyRequest(() -> autoNote.withVelocityX(1).withRotationalRate(photon.noteAutoAimRateOutput()));

    }

    else {
      drivetrain.applyRequest(() -> RobotContainer.drive
          .withVelocityX(CommandSwerveDrivetrain
              .getExponential(-RobotContainer.m_driver_controler.getLeftY() * RobotContainer.MaxSpeed)) // Drive forward
                                                                                                        // with
          // negative Y (forward)
          .withVelocityY(CommandSwerveDrivetrain
              .getExponential(-RobotContainer.m_driver_controler.getLeftX() * RobotContainer.MaxSpeed)) // Drive left
                                                                                                        // with negative
                                                                                                        // X (left)
          .withRotationalRate(CommandSwerveDrivetrain
              .getExponential(-RobotContainer.m_driver_controler.getRightX() * RobotContainer.MaxAngularRate)) // Drive
                                                                                                               // counterclockwise
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                                                                                               // (left)
      );

      intake.stopIntake();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
