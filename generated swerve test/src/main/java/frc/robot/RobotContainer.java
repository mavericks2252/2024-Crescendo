// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

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
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AmpShootNote;
import frc.robot.commands.AutoAimCornerShot;
import frc.robot.commands.AutoAimManualAngle;
import frc.robot.commands.AutoAimShootNote;
import frc.robot.commands.AutoNoteIntake;
import frc.robot.commands.AutoShooterSpool;
import frc.robot.commands.AutonomousNoteIntake;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeAndShoot;
import frc.robot.commands.IntakeAndShootcopy;
import frc.robot.commands.IntakeBackwards;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntakeStage;
import frc.robot.commands.ManualAmpShot;
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
        public static double MaxSpeed = 4.5; // 6 meters per second desired top speed
        public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        public static final CommandXboxController m_driver_controler = new CommandXboxController(0); // My
                                                                                                     // m_driver_controler
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.05)
                        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                                 // driving in open loop
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final Telemetry logger = new Telemetry(MaxSpeed);

        // Robot Subsystems
        public final static Shooter shooter = new Shooter();
        public final static Intake intake = new Intake();
        public final VisionPhotonSubsystem visionPhotonSubsystem = new VisionPhotonSubsystem(drivetrain);
        public final ShooterRotationSubsystem shooterRotationSubsystem = new ShooterRotationSubsystem(
                        visionPhotonSubsystem);
        public final static LEDSubsystem ledSubsystem = new LEDSubsystem();
        public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

        // Robot Commands
        public final static CommandXboxController m_operatorController = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        private final SendableChooser<Command> autoChooser;

        private final SendableChooser<Command> testChooser;

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
                                                                                                .getRotationalExponential(
                                                                                                                -m_driver_controler
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
                m_driver_controler.x().toggleOnTrue(new SequentialCommandGroup(new ParallelCommandGroup(

                                new SetAmpMode(intake, shooterRotationSubsystem, shooter,
                                                visionPhotonSubsystem),

                                drivetrain.pathfinding("Amp Score")),
                                new AmpShootNote(intake, shooterRotationSubsystem, shooter,
                                                visionPhotonSubsystem)));

                m_driver_controler.povLeft().toggleOnFalse(new SequentialCommandGroup(
                                new InstantCommand(() -> shooterRotationSubsystem.setClimbMode()),
                                drivetrain.pathfinding("AutoClimbLeft")));
                m_driver_controler.povRight().toggleOnFalse(new SequentialCommandGroup(
                                new InstantCommand(() -> shooterRotationSubsystem.setClimbMode()),
                                drivetrain.pathfinding("AutoClimbRight")));
                m_driver_controler.povUp().toggleOnFalse(new SequentialCommandGroup(
                                new InstantCommand(() -> shooterRotationSubsystem.setClimbMode()),
                                drivetrain.pathfinding("AutoClimbBack")));
                m_driver_controler.a().toggleOnTrue(new AutoAimCornerShot(shooter, shooterRotationSubsystem,
                                visionPhotonSubsystem, drivetrain));

                // Operator Buttons
                // manual speaker shot
                m_operatorController.rightBumper()
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> shooterRotationSubsystem.setManualShoot()),
                                                new InstantCommand(() -> shooterRotationSubsystem
                                                                .setShooterAngle(ShooterConstants.kAmpAngle))));

                m_operatorController.y().whileTrue(new ManualShootNote(shooter, shooterRotationSubsystem));

                // manual amp shot
                m_operatorController.leftBumper().whileTrue(
                                new ManualAmpShot(shooter));

                // run intake backwards
                m_operatorController.x()
                                .whileTrue(new IntakeBackwards(intake, shooter));

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
                                new ShootNote(shooter, shooterRotationSubsystem, visionPhotonSubsystem, 134.75));

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

                NamedCommands.registerCommand("AutoSpool", new SequentialCommandGroup(
                                new IntakeStage(shooter, intake),
                                new InstantCommand(() -> shooter.setShooterVelocity(3000))));

                NamedCommands.registerCommand("fixedAngleShot", new IntakeAndShootcopy(intake,
                                shooter,
                                shooterRotationSubsystem,
                                visionPhotonSubsystem));

                NamedCommands.registerCommand("AutoAimManualAngle", new AutoAimManualAngle(shooter,
                                shooterRotationSubsystem, visionPhotonSubsystem, drivetrain, 113));

                NamedCommands.registerCommand("AutoAimManual2", new AutoAimManualAngle(shooter,
                                shooterRotationSubsystem, visionPhotonSubsystem, drivetrain, 111.5));

                NamedCommands.registerCommand("IntakeStage", new IntakeStage(shooter, intake));

                /*
                 * NamedCommands.registerCommand("AutonomousIntake", new
                 * AutonomousNoteIntake(visionPhotonSubsystem,
                 * intake, drivetrain, shooterRotationSubsystem, shooter));
                 */

                NamedCommands.registerCommand("AutonomousIntake",
                                new SequentialCommandGroup(
                                                new AutonomousNoteIntake(visionPhotonSubsystem, intake, drivetrain,
                                                                shooterRotationSubsystem, shooter),
                                                new IntakeNote(intake, shooterRotationSubsystem, shooter, ledSubsystem),
                                                new IntakeStage(shooter, intake)).withTimeout(2));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                testChooser = new SendableChooser<>();
                testChooser.addOption("four note source side", Autos.fourNoteSourceSideAuto(shooter, drivetrain));

                NamedCommands.registerCommand("IntakeMode",
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> shooterRotationSubsystem.setIntakeMode()),
                                                new InstantCommand(() -> shooterRotationSubsystem
                                                                .setShooterIntakeAngle())));
        }

        public Command getAutonomousCommand() {

                return autoChooser.getSelected();

                // return Commands.print("No autonomous command configured");
        }

}
