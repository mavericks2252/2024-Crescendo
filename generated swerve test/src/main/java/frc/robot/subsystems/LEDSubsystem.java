// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class LEDSubsystem extends SubsystemBase {
  public boolean speakerMode = true;
  Spark blinkin;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {

    blinkin = new Spark(0);
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   SmartDashboard.putBoolean("SpeakerMode", speakerMode);
    setLEDColor(BlinkinConstants.kFire);
  }

    public void setLEDColor(double color){

      blinkin.set(color);

    }


    
      //Speaker mode = true is speaker mode
      //Speaker mode = false is amp mode
    public void setSpeakerMode(){
      
      if(speakerMode){
        blinkin.set(BlinkinConstants.kBlue);
        speakerMode = false;
      }
      else{
        blinkin.set(BlinkinConstants.kRed);
        speakerMode = true;
      }
    }


}