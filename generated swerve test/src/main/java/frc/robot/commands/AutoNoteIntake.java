// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
//import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class AutoNoteIntake extends Command {
  /** Creates a new NoteAim. */
  VisionPhotonSubsystem photon;
  Intake intake;
  CommandSwerveDrivetrain drivetrain;
  ShooterRotationSubsystem shooterRotationSubsystem;
  Shooter shooter;
  int loopsWithoutTarget;
  double forwardSpeed;

  // AutoAimSubsystem autoAimSubsystem;

  private final SwerveRequest.RobotCentric autoNote = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(RobotContainer.MaxSpeed * 0.1).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a
                                                                                                               // 10%
                                                                                                               // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoNoteIntake(VisionPhotonSubsystem photon,
      Intake intake,
      CommandSwerveDrivetrain drivetrain,
      ShooterRotationSubsystem shooterRotationSubsystem,
      Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photon = photon;
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.shooter = shooter;
    // this.autoAimSubsystem = autoAimSubsystem;

    addRequirements(intake, drivetrain, shooterRotationSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopsWithoutTarget = 11;
    shooterRotationSubsystem.setShooterIntakeAngle();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_driver_controler.getHID().setRumble(RumbleType.kBothRumble, 1);

    if (photon.noteCam.getLatestResult().hasTargets()) {
      loopsWithoutTarget = 0;
    } else {
      loopsWithoutTarget++;
    }

    SmartDashboard.putNumber("loops without target", loopsWithoutTarget);

    if (photon.noteCam.getLatestResult().hasTargets() || loopsWithoutTarget < 10) {
      intake.setIntakeSpeed();
      LEDSubsystem.orange();

      if (photon.getNoteTargetArea() > 3) {
        forwardSpeed = -1;
      } else
        forwardSpeed = -3;

      // drivetrain.applyRequest(() ->
      // autoNote.withVelocityX(1).withRotationalRate(photon.noteAutoAimRateOutput()));
      drivetrain.setControl(autoNote.withVelocityX(forwardSpeed).withRotationalRate(photon.noteAutoAimRateOutput()));

    }

    else {
      LEDSubsystem.blue();
      drivetrain.setControl(drive
          .withVelocityX(CommandSwerveDrivetrain
              .getExponential(-RobotContainer.m_driver_controler.getLeftY() * RobotContainer.MaxSpeed)) // Drive forward
                                                                                                        // with
          // negative Y (forward)
          .withVelocityY(CommandSwerveDrivetrain
              .getExponential(-RobotContainer.m_driver_controler.getLeftX() * RobotContainer.MaxSpeed)) // Drive left
                                                                                                        // with negative
                                                                                                        // X (left)
          .withRotationalRate(CommandSwerveDrivetrain
              .getRotationalExponential(-RobotContainer.m_driver_controler.getRightX() * RobotContainer.MaxAngularRate)) // Drive
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
    drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    LEDSubsystem.green();
    RobotContainer.m_driver_controler.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intake.getBeamBreak() || shooter.getMiddleBackBeambreak()) {
      return true;
    } else
      return false;
  }
}
