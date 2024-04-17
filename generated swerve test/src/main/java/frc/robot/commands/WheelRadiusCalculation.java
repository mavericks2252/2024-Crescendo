// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;

public class WheelRadiusCalculation extends Command {

  CommandSwerveDrivetrain drivetrain;
  private static double driveBaseRadius;
  private static DoubleSupplier gyroYawSupplier;
  private final SlewRateLimiter omegSlewRateLimiter = new SlewRateLimiter(1);

  private double lastGyroYaw = 0.0;
  private double accumGyroYaw = 0.0;

  private double[] wheelStartPositions;
  private double currentWheelEffectiveRadius = 0.0;

  private SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

  /** Creates a new WheelRadiusCalculation. */
  public WheelRadiusCalculation(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    driveBaseRadius = drivetrain.getDriveBaseRadius();
    gyroYawSupplier = () -> Units.degreesToRadians(drivetrain.getPigeon2().getYaw().getValueAsDouble());

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // set last gyro reading as current reading
    lastGyroYaw = gyroYawSupplier.getAsDouble();
    // zero out the accumulated gyro values
    accumGyroYaw = 0.0;

    // get and set all the starting wheel positions in radians
    wheelStartPositions = drivetrain.getWheelPosition();

    omegSlewRateLimiter.reset(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // set the drivetrain to rotate at a given speed with a rate limiting
    drivetrain.setControl(drive.withRotationalRate(omegSlewRateLimiter.calculate(2)));

    // add the current gyro minus the last gyro reading to the accumulated value
    accumGyroYaw += MathUtil.angleModulus(gyroYawSupplier.getAsDouble() - lastGyroYaw);
    // set last reading to current reading since we already used it
    lastGyroYaw = gyroYawSupplier.getAsDouble();

    double averageWheelPosition = 0;
    double[] wheelPositionsRads = drivetrain.getWheelPosition();
    // average the wheel positions taken
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositionsRads[i] - wheelStartPositions[i]);
    }
    averageWheelPosition /= 4.0;

    // calculate the current wheel radius
    currentWheelEffectiveRadius = (accumGyroYaw * driveBaseRadius) / averageWheelPosition;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (accumGyroYaw <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentWheelEffectiveRadius)
              + " inches");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
