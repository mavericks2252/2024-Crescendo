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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpShootNote;
import frc.robot.commands.AutoAimShootNote;
import frc.robot.commands.AutoNoteIntake;
import frc.robot.commands.AutoShooterSpool;
import frc.robot.commands.IntakeAndShoot;
import frc.robot.commands.IntakeAndShootcopy;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntakeStage;
import frc.robot.commands.ManualShootNote;
import frc.robot.commands.SetAmpMode;
import frc.robot.commands.ShootNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
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
        public static final CommandXboxController m_driver_controler = new CommandXboxController(0); // My
                                                                                                     // m_driver_controler
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
        public final VisionPhotonSubsystem visionPhotonSubsystem = new VisionPhotonSubsystem(drivetrain);
        public final ShooterRotationSubsystem shooterRotationSubsystem = new ShooterRotationSubsystem(
                        visionPhotonSubsystem);
        public final LEDSubsystem ledSubsystem = new LEDSubsystem(shooter, intake, visionPhotonSubsystem,
                        shooterRotationSubsystem);
        public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

        // Robot Commands
        public final static CommandXboxController m_operatorController = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        private final SendableChooser<Command> autoChooser;

        private void configureBindings() {

                climberSubsystem.setDefaultCommand(
                                climberSubsystem.applyRequests(() -> -m_operatorController.getLeftY()));

                shooter.setDefaultCommand(
                                new AutoShooterSpool(shooter, visionPhotonSubsystem, shooterRotationSubsystem));

                // Swerve Buttons
                drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(
                                                () -> drive
                                                                .withVelocityX(CommandSwerveDrivetrain
                                                                                .getExponential(-m_driver_controler
                                                                                                .getLeftY() * MaxSpeed)) // Drive
                                                                                                                         // forward
                                                                                                                         // with
                                                                // negative Y (forward)
                                                                .withVelocityY(CommandSwerveDrivetrain
                                                                                .getExponential(-m_driver_controler
                                                                                                .getLeftX() * MaxSpeed)) // Drive
                                                                                                                         // left
                                                                                                                         // with
                                                                                                                         // negative
                                                                                                                         // X
                                                                                                                         // (left)
                                                                .withRotationalRate(
                                                                                CommandSwerveDrivetrain
                                                                                                .getExponential(-m_driver_controler
                                                                                                                .getRightX()
                                                                                                                * MaxAngularRate)) // Drive
                                                                                                                                   // counterclockwise
                                                                                                                                   // with
                                                                                                                                   // negative
                                                                                                                                   // X
                                                                                                                                   // (left)
                                ));

                m_driver_controler.a().whileTrue(drivetrain.applyRequest(() -> brake));
                m_driver_controler.b().whileTrue(drivetrain
                                .applyRequest(() -> point
                                                .withModuleDirection(
                                                                new Rotation2d(-m_driver_controler.getLeftY(),
                                                                                -m_driver_controler.getLeftX()))));

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

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
                drivetrain.registerTelemetry(logger::telemeterize);

                // Driver Buttons
                // reseed field orientation
                m_driver_controler.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

                // auto aim and shoot
                m_driver_controler.leftBumper()
                                .onTrue(new AutoAimShootNote(shooter, shooterRotationSubsystem, visionPhotonSubsystem,
                                                drivetrain));

                // intake note
                m_driver_controler.rightBumper()
                                .toggleOnTrue(new SequentialCommandGroup(
                                                new IntakeNote(intake, shooterRotationSubsystem, shooter, ledSubsystem),
                                                new IntakeStage(shooter, intake)));

                // auto aim and intake note
                m_driver_controler.rightTrigger()
                                .toggleOnTrue(new SequentialCommandGroup(
                                                new AutoNoteIntake(visionPhotonSubsystem, intake, drivetrain,
                                                                shooterRotationSubsystem, shooter),
                                                new IntakeNote(intake, shooterRotationSubsystem, shooter, ledSubsystem),
                                                new IntakeStage(shooter, intake)));

                // auto amp path and shot
                m_driver_controler.x().onTrue(new SequentialCommandGroup(new ParallelCommandGroup(

                                new SetAmpMode(intake, shooterRotationSubsystem, shooter,
                                                visionPhotonSubsystem),

                                drivetrain.AmpPathfinding()),
                                new AmpShootNote(intake, shooterRotationSubsystem, shooter,
                                                visionPhotonSubsystem)));

                // Operator Buttons
                // manual speaker shot
                m_operatorController.y().whileTrue(new ManualShootNote(shooter, shooterRotationSubsystem));

                // manual amp shot
                m_operatorController.leftBumper().toggleOnTrue(new ParallelCommandGroup(
                                new InstantCommand(() -> shooterRotationSubsystem.setShooterAmpAngle()),
                                new AmpShootNote(intake, shooterRotationSubsystem, shooter, visionPhotonSubsystem)));

                // run intake backwards
                m_operatorController.x()
                                .whileTrue(new ParallelCommandGroup(
                                                new InstantCommand(() -> intake.setIntakeBackwards()),
                                                new InstantCommand(() -> shooter.SetIntakeWheelsBack())));

                // set climbing mode on left joystick click
                m_operatorController.leftStick()
                                .onTrue(new InstantCommand(() -> shooterRotationSubsystem.setClimbMode()));

        }

        public RobotContainer() {
                configureBindings();

                // named commands
                NamedCommands.registerCommand("IntakeNote",
                                new IntakeNote(intake, shooterRotationSubsystem, shooter, ledSubsystem)
                                                .withTimeout(3));
                NamedCommands.registerCommand("ShootNote",
                                new ShootNote(shooter, shooterRotationSubsystem, visionPhotonSubsystem));

                NamedCommands.registerCommand("AmpShootNote",
                                new AmpShootNote(intake, shooterRotationSubsystem, shooter, visionPhotonSubsystem));
                NamedCommands.registerCommand("AutoAimShot",
                                new AutoAimShootNote(shooter, shooterRotationSubsystem, visionPhotonSubsystem,
                                                drivetrain));
                NamedCommands.registerCommand("AutoNoteIntake",
                                new SequentialCommandGroup(
                                                new AutoNoteIntake(visionPhotonSubsystem, intake, drivetrain,
                                                                shooterRotationSubsystem, shooter),
                                                new IntakeNote(intake, shooterRotationSubsystem, shooter, ledSubsystem),
                                                new IntakeStage(shooter, intake)));
                NamedCommands.registerCommand("IntakeAndShoot",
                                new IntakeAndShoot(intake, shooter, shooterRotationSubsystem, visionPhotonSubsystem));

                NamedCommands.registerCommand("AutoSpool", new InstantCommand(() -> shooter.setShooterVelocity(3000)));

                NamedCommands.registerCommand("fixedAngleShot", new IntakeAndShootcopy(intake,
                                shooter,
                                shooterRotationSubsystem,
                                visionPhotonSubsystem));

                NamedCommands.registerCommand("IntakeStage", new IntakeStage(shooter, intake));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        public Command getAutonomousCommand() {

                return autoChooser.getSelected();

                // return Commands.print("No autonomous command configured");
        }

}
