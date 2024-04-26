// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class LEDSubsystem extends SubsystemBase {

  public static Spark blinkin;

  public enum LEDState {
    INTAKING(.87), // blue
    HAS_NOTE(.77), // green
    AUTO_INTAKE(-.09), // strobe blue
    AMP_MODE(.61), // red
    INTAKE_COMPLETED(-.05),
    OFF(0.0),
    STROBE_WHITE(-.02);

    private double value;

    LEDState(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }

  private static LEDState currentState = LEDState.OFF;
  private static LEDState lastState = LEDState.OFF;

  private static Timer blinkTimer = new Timer();
  private static boolean blinkRequested = false;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {

    blinkin = new Spark(9);

  }

  @Override
  public void periodic() {

    setLEDs();

  }

  public static void setLEDState(LEDState state) {
    currentState = state;
  }

  public void setLEDs() {
    if (currentState != lastState && currentState == LEDState.HAS_NOTE) {
      blinkRequested = true;
      blinkTimer.restart();
    }

    if (blinkRequested && blinkTimer.get() < 2)
      blinkin.set(LEDState.STROBE_WHITE.getValue());
    else {
      blinkRequested = false;
      blinkin.set(currentState.getValue());
    }

    lastState = currentState;
  }

  // older code
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
