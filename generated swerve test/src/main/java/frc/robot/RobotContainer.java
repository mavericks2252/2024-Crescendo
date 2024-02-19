// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAimShootNote;
import frc.robot.commands.AutoNoteIntake;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRotationSubsystem;
import frc.robot.subsystems.VisionPhotonSubsystem;

public class RobotContainer {

  // Swerve Stuff
  public static double MaxSpeed = 2.5; // 6 meters per second desired top speed
  public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final CommandXboxController m_driver_controler = new CommandXboxController(0); // My m_driver_controler
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Robot Subsystems
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem();
  public final VisionPhotonSubsystem visionPhotonSubsystem = new VisionPhotonSubsystem(drivetrain);
  public final ShooterRotationSubsystem shooterRotationSubsystem = new ShooterRotationSubsystem(visionPhotonSubsystem);

  // Robot Commands
  public final static CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  private void configureBindings() {

    // Swerve Buttons
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> drive.withVelocityX(CommandSwerveDrivetrain.getExponential(-m_driver_controler.getLeftY() * MaxSpeed)) // Drive
                                                                                                                         // forward
                                                                                                                         // with
                // negative Y (forward)
                .withVelocityY(CommandSwerveDrivetrain.getExponential(-m_driver_controler.getLeftX() * MaxSpeed)) // Drive
                                                                                                                  // left
                                                                                                                  // with
                                                                                                                  // negative
                                                                                                                  // X
                                                                                                                  // (left)
                .withRotationalRate(
                    CommandSwerveDrivetrain.getExponential(-m_driver_controler.getRightX() * MaxAngularRate)) // Drive
                                                                                                              // counterclockwise
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // X
                                                                                                              // (left)
        ));

    m_driver_controler.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driver_controler.b().whileTrue(drivetrain
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-m_driver_controler.getLeftY(), -m_driver_controler.getLeftX()))));

    m_driver_controler.x()
        .onTrue(new AutoAimShootNote(shooter, shooterRotationSubsystem, visionPhotonSubsystem, drivetrain));

    /*
     * m_driver_controler.x()
     * .whileTrue(drivetrain.applyRequest(() ->
     * autoAimDrive.withVelocityX(-m_driver_controler.getLeftY() * MaxSpeed) //
     * Drive
     * // forward
     * // with
     * // negative Y (forward)
     * .withVelocityY(-m_driver_controler.getLeftX() * MaxSpeed) // Drive left with
     * negative X (left)
     * .withRotationalRate(
     * visionPhotonSubsystem.speakerAutoAimRateOutput()) // Drive counterclockwise
     * with negative X (left)
     * ));
     */

    // reset the field-centric heading on left bumper press
    m_driver_controler.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Driver Buttons

    m_driver_controler.start().onTrue(new InstantCommand(() -> visionPhotonSubsystem.seedRobotPoseFromVision()));
    m_driver_controler.rightBumper().toggleOnTrue(new IntakeNote(intake, shooterRotationSubsystem, shooter));
    m_driver_controler.a()
        .whileTrue(new AutoNoteIntake(visionPhotonSubsystem, intake, drivetrain, shooterRotationSubsystem));
    m_driver_controler.y().onTrue(drivetrain.ampPathCommand());

    // Operator Buttons

    m_operatorController.b()
        .whileTrue(new ShootNote(shooter, shooterRotationSubsystem, visionPhotonSubsystem));

    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> ledSubsystem.setSpeakerMode()));

    m_operatorController.y().onTrue(new InstantCommand(() -> shooterRotationSubsystem.setSpeakerTracking()));
    m_operatorController.x().onTrue(new InstantCommand(() -> shooterRotationSubsystem.setAmpMode()));
    m_operatorController.a().onTrue(new InstantCommand(() -> shooterRotationSubsystem.setIntakeMode()));

  }

  public RobotContainer() {
    configureBindings();

    // named commands
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake, shooterRotationSubsystem, shooter));
    NamedCommands.registerCommand("ShootNote", new ShootNote(shooter, shooterRotationSubsystem, visionPhotonSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

    // return Commands.print("No autonomous command configured");
  }

}
