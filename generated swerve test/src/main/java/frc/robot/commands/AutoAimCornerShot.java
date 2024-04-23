// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class AutoAimCornerShot extends Command {
  /** Creates a new AutoAimCornerShot. */
  Shooter shooter;
  ShooterRotationSubsystem shooterRotationSubsystem;
  VisionPhotonSubsystem photon;
  RobotContainer robotContainer;
  CommandSwerveDrivetrain drivetrain;
  double targetRPM;

  private final SwerveRequest.FieldCentric autoAim = new SwerveRequest.FieldCentric()
      .withDeadband(RobotContainer.MaxSpeed * 0.15).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoAimCornerShot(Shooter shooter, ShooterRotationSubsystem shooterRotationSubsystem,
      VisionPhotonSubsystem photon, CommandSwerveDrivetrain drivetrain) {
    this.shooter = shooter;
    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.photon = photon;
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, shooterRotationSubsystem, shooter, photon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterRotationSubsystem.setManualShoot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterRotationSubsystem.setShooterAngle(130);
    double error = Math.abs((photon.cornerAutoAimRateOutput()));
    targetRPM = 2800;
    shooter.setShooterVelocity(targetRPM);
    SmartDashboard.putNumber("corner shot error", error);
    // shooterRotationSubsystem.setShooterAngle(photon.getTargetAngle());

    if (targetRPM - shooter.getShooterVelocity() < 200 && error < .75) {
      shooter.acceleratorWheelOutput(1);
      shooter.setAmpWheel(1);
    } else {
      shooter.acceleratorWheelOutput(0);
      shooter.setAmpWheel(0);
    }

    drivetrain.setControl(autoAim.withVelocityX(-RobotContainer.m_driver_controler.getLeftY() * 0.75) // Drive
                                                                                                      // forward
                                                                                                      // with
        // negative Y (forward)
        .withVelocityY(-RobotContainer.m_driver_controler.getLeftX() * 0.75) // Drive left with negative X (left)
        .withRotationalRate(
            photon.cornerAutoAimRateOutput()));
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
