// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  
  public static final class PortConstants {

    
    public static final int kIntakeMotorMasterPort = 15;
    public static final int kPneumaticForward = 1;
    public static final int kPneumaticReverse = 0;
    public static final int kShooterMotorSlavePort = 18;
    public static final int kShooterMotorMasterPort = 17;
    public static final int kIntakeMotorSlavePort = 16;
    public static final int kFeedMotorPort = 19;
    public static final int kFeedMotorSlavePort = 20;
  }
  
  public static final class IntakeConstants {

    public static final int kBeamBreak = 0;
    public static final double kIntakeMasterSpeed = 1;
    public static final double kIntakeSlaveSpeed = 1;

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
    
  }


     
}
