// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class PortConstants {

    public static final int kIntakeMotorPort = 15;
    public static final int kShooterMotorSlavePort = 18;
    public static final int kShooterMotorMasterPort = 17;
    public static final int kAcceleratorWheelPort = 20;
    public static final int kFeedMotorPort = 21;
    public static final int kacceleratorWheelSlavePort = 19;
    public static final int kShooterAngleMotorPort = 22;
    public static final int kAmplifierWheelPort = 23;
    public static final int kCenteringWheelPort = 25;

    public static final int kShotBeamBreak = 1;
    public static final int kAmpBeamBreak = 2;
    public static final int kMiddleBackBeamBreak = 3;
    public static final int kMiddleFrontBeamBreak = 4;
    public static final int kThroughBoreEncoder = 5;
  }

  public static final class IntakeConstants {

    public static final int kBeamBreak = 0;
    public static final double kIntakeSpeed = 1;
    public static final double kCenteringWheelSpeed = 1;
    public static final double kAcceleratorWheelSpeed = 0.2625;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorSpeed = 4200;
    public static final double kacceleratorWheelSpeed = 0.8;
    public static final double kIntakeSpeed = 1;
    public static final double kIntakeAngle = 135;
    public static final double kAmpAngle = 8;
    public static final double kShooterGearBoxRatio = 270.830864;
    public static final int kForwardSoftLimit = 140;
    public static final int kReverseSoftLimit = 5;
  }

  public static final class OIConstants {

    public static final double kDeadBand = .15;
    public static final int kOperatorControllerPort = 1;
    public static final int kDriverControllerport = 0;
    public static final int kDriverXAxis = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 4;
    public static final int kRTrigger = 3;
    public static final int kLTrigger = 2;

    // Controller Buttons
    public static final int xButton = 3;
    public static final int aButton = 1;
    public static final int bButton = 2;
    public static final int yButton = 4;
    public static final int rbButton = 6;
    public static final int lbButton = 5;
    public static final int menuButton = 8;
    public static final int viewButton = 7;
  }

  public final class BlinkinConstants {

    public static final double kRed = 0.61;
    public static final double kWhite = 0.93;
    public static final double kGold = 0.67;
    public static final double kBlue = 0.87;
    public static final double kGreen = 0.77;
    public static final double kBlueHB = -0.23;
    public static final double kBlueShot = -0.83;
    public static final double kFire = -0.57;
    public static final double kRaindow = -0.99;

  }

  public final class FieldConstants {

    public static final double kBlueSpeakerXPos = 0;
    public static final double kBlueSpeakerYPos = 218.42;
    public static final double kBlueSpeakerXPosMeters = 0;
    public static final double kBlueSpeakerYPosMeters = 5.547;
    public static final Translation2d kBlueSpeaker = new Translation2d(kBlueSpeakerXPosMeters, kBlueSpeakerYPosMeters);

    public static final double kRedSpeakerXPosMeters = 15.64;
    public static final double kRedSpeakerYPosMeters = 5.547;
    public static final Translation2d kRedSpeaker = new Translation2d(kRedSpeakerXPosMeters, kRedSpeakerYPosMeters);

    public static final double kSpeakerHeight = 1.981;
    public static final double kHeightToPivot = 0.4191; // 0.482

    public static final double kPivotToSpeaker = kSpeakerHeight - kHeightToPivot;

    public static final double kRobotOffsetMeters = .5;
    public static final double kBlueAmpXPosMeters = 1.841;
    public static final double kBlueAmpYPosMeters = 8.204; // 8.204
    public static final Pose2d kBlueAmp = new Pose2d(kBlueAmpXPosMeters, kBlueAmpYPosMeters,
        Rotation2d.fromDegrees(90));
    public static final Pose2d kBlueAmpScorePose = new Pose2d(kBlueAmpXPosMeters,
        kBlueAmpYPosMeters - kRobotOffsetMeters,
        Rotation2d.fromDegrees(90));
    public static final double kInfrontBlueAmpYPos = kBlueAmpYPosMeters - 0.5 - kRobotOffsetMeters;
    public static final Pose2d kInfrontBluePos = new Pose2d(kBlueAmpXPosMeters, kInfrontBlueAmpYPos,
        new Rotation2d(90));

    public static final double kRedAmpXPosMeters = 14.7;
    public static final double kRedAmpYPosMeters = 8.204;
    public static final Pose2d kRedAmp = new Pose2d(kRedAmpXPosMeters, kRedAmpYPosMeters, Rotation2d.fromDegrees(90));
    public static final double kInfrontRedAmpYPos = kRedAmpYPosMeters - .5;
    public static final Pose2d kInfrontRedPos = new Pose2d(kRedAmpXPosMeters, kInfrontRedAmpYPos, new Rotation2d(90));
  }

  public final class DriveTrainConstants {
    public static final double kExponent = 3;
    public static final double kWeight = 0.75;
    public static final double kDeadBand = 0.1;

    public static final PathConstraints kPathConstraints = new PathConstraints(
        2, // max Velocity 3
        2, // max Acceleration 3
        720, // max angular velocity 720
        540); // max angular acceleration 540

  }

  public final class VisionConstants {
    public static final String kCameraName = "FrontAprilTagCam";
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final int kApriltagePipeline = 0;
    public static final int kNotePipeline = 1;

    public static final Transform3d kRobotToCam = new Transform3d(

        new Translation3d(Units.inchesToMeters(15.75),
            Units.inchesToMeters(-11.75),
            Units.inchesToMeters(9)),

        new Rotation3d(0, Units.degreesToRadians(-34), 0));
  }

}
