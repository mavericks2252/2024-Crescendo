// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.BlinkinConstants;

public class LEDSubsystem extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;

  Boolean hasNote;
  boolean seesNote;
  public Boolean isIntaking;

  public static Spark blinkin;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {

    blinkin = new Spark(9);

  }

  @Override
  public void periodic() {

  }

  public static void teamLEDColor() {
    if (DriverStation.getAlliance().isPresent()) { // if we are getting an alliance color
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) // if its blue
        blinkin.set(BlinkinConstants.kBlue); // make the lights blue
      else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)// if its red
        blinkin.set(BlinkinConstants.kRed); // make the lights red
    }
  }

  public static void orange() {
    blinkin.set(BlinkinConstants.kOrange);
  }

  public static void green() {
    blinkin.set(BlinkinConstants.kGreen);
  }

  public static void red() {
    blinkin.set(BlinkinConstants.kRed);
  }

  public static void blue() {
    blinkin.set(BlinkinConstants.kBlue);
  }

}
