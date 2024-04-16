// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command fourNoteSourceSideAuto(Shooter shooter, CommandSwerveDrivetrain drivetrain) {

    return Commands.sequence(
        new PathPlannerAuto("Shoot Pre Load and Get 2nd Note"), // runs an initial command to shoot note and follow path
                                                                // to get a note
        Commands.either(
            new PathPlannerAuto("score 2nd note and get 3rd"), // run this auto if it got the second note
            new PathPlannerAuto("missed second get 3rd note"), // run this auto if it missed the second note
            shooter::hasAutoNote), // boolean supplier for if the robot has a note
        Commands.sequence(new ShootNote(null, null, null, 0))); // command to shoot a note or whatever might be needed

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
