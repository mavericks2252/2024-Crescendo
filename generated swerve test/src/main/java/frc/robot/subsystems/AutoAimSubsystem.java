// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class AutoAimSubsystem extends SubsystemBase {
  TrapezoidProfile.Constraints aim_PIDConstraints = new TrapezoidProfile.Constraints(TunerConstants.kMaxAngularRate,
      TunerConstants.kMaxAngularAcceleration);

  ProfiledPIDController autoAimPIDController;
  ProfiledPIDController noteAimPidController;
  // VisionSubsystem vision;
  VisionPhotonSubsystem photon;

  // Creates a new AutoAimSubsystem.
  public AutoAimSubsystem(VisionPhotonSubsystem photon, LEDSubsystem ledSubsystem) {
    // this.vision = vision;
    this.photon = photon;

    noteAimPidController = new ProfiledPIDController(3, 0.25, 0, aim_PIDConstraints, .01);
    noteAimPidController.enableContinuousInput(-Math.PI, Math.PI);
    noteAimPidController.setTolerance(Units.degreesToRadians(1));

    autoAimPIDController = new ProfiledPIDController(5, 0.25, 0, aim_PIDConstraints, .01);
    autoAimPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // autoAimPIDController.setTolerance(3);
    autoAimPIDController.setIZone(.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("auto aim speaker output",
    // speakerAutoAimRateOutput());
    // SmartDashboard.putNumber("auto aim note output", noteAutoAimRateOutput());

  }

  public double speakerAutoAimRateOutput() {
    Pose2d currentPos = photon.getCurrentPose2d(); // gets the current robot position
    Pose2d targetPos = photon.getSpeakerTargetRotation2d(); // gets the rotation need to aim at the speaker

    return autoAimPIDController.calculate(currentPos.getRotation().getRadians(), targetPos.getRotation().getRadians());
    // takes the current pose and its target pose and calculates the rotation to get
    // there

  }

  public double noteAutoAimRateOutput() {
    // double currentRotation =
    // photon.getCurrentPose2d().getRotation().getDegrees();
    double angleToNote;
    double turnRate;
    var target = photon.camera.getLatestResult(); // sets "target" to the closest note on the field

    if (target.hasTargets()) { // if there is a note
      angleToNote = target.getBestTarget().getYaw(); // finds the rotation needed to get to the note
      turnRate = noteAimPidController.calculate(Units.degreesToRadians(angleToNote)); // returns the calculation needed
                                                                                      // to reach the note in radians
    } else { // if there isn't a note
      angleToNote = 0; // set the rotation needed to 0
      turnRate = CommandSwerveDrivetrain
          .getRotationalExponential(-RobotContainer.m_driver_controler.getRightX() * RobotContainer.MaxAngularRate); // sets
                                                                                                                     // the
      // control to
      // the driver
      // controller
    }
    // SmartDashboard.putNumber("angle to note", turnRate);
    return turnRate;

  }

}
