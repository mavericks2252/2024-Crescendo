// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class AutoAimManualAngle extends Command {
  /** Creates a new ShootNote. */
  Shooter shooter;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;
  RobotContainer robotContainer;
  CommandSwerveDrivetrain drivetrain;
  double targetAngle;

  double targetRPM, maxSpeed = 0.75;

  private final SwerveRequest.FieldCentric autoAimDrive = new SwerveRequest.FieldCentric()
      .withDeadband(RobotContainer.MaxSpeed * 0.15).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoAimManualAngle(Shooter shooter, ShooterRotationSubsystem shooterRotationSubsystem,
      VisionPhotonSubsystem photon, CommandSwerveDrivetrain drivetrain, double targetAngle) {
    this.shooter = shooter;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.photon = photon;
    this.drivetrain = drivetrain;
    this.targetAngle = targetAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, shooterRotationSubsystem, drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterRotationSubsystem.setManualShoot();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = Math.abs((photon.speakerAutoAimRateOutput()));
    targetRPM = photon.getTargetRPM();
    shooterRotationSubsystem.setShooterAngle(targetAngle);

    if (targetRPM - shooter.getShooterVelocity() < 75 && error < .75) {
      shooter.acceleratorWheelOutput(1);
    }

    drivetrain.setControl(autoAimDrive.withVelocityX(-RobotContainer.m_driver_controler.getLeftY() * maxSpeed) // Drive
                                                                                                               // forward
                                                                                                               // with
        // negative Y (forward)
        .withVelocityY(-RobotContainer.m_driver_controler.getLeftX() * maxSpeed) // Drive left with negative X (left)
        .withRotationalRate(
            photon.speakerAutoAimRateOutput()));

    shooter.setShooterVelocity(targetRPM);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.stopShooter();
    shooter.stopAcceleratorWheel();
    shooter.stopAmplifierWheel();
    shooterRotationSubsystem.setIntakeMode();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!shooter.getMiddleBackBeambreak() && !shooter.getShotBeambreak() && !shooter.getMiddleFrontBeambreak()) {
      return true;
    } else {
      return false;
    }
  }
}
