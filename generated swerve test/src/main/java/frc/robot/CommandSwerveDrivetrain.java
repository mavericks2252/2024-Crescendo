package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    Pose2d ampPos = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d infrontAmpPos = new Pose2d(0, 0, new Rotation2d(0));
    private static final SlewRateLimiter rateLimeter = new SlewRateLimiter(12);

    public final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds(); // creates a
                                                                                                        // swerve
                                                                                                        // request
                                                                                                        // called auto
                                                                                                        // request

    public final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(TunerConstants.kmaxSpeed * .1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configPathPlanner();
        setHeadingPID();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configPathPlanner();
        setHeadingPID();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // method for getting the drivebase radius
    public double getDriveBaseRadius() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
            // finds the radius by taking the larger of the locations
        }
        return driveBaseRadius;
    }

    public HolonomicPathFollowerConfig getHolonomicPathFollowerConfig() {
        double driveBaseRadius = getDriveBaseRadius();

        return new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0), // constants for the translation controller
                new PIDConstants(10, 0, 0), // constants for the rotation controller
                TunerConstants.kSpeedAt12VoltsMps, // sets the max speed
                driveBaseRadius,
                new ReplanningConfig());

    }

    // Configure auto builder
    private void configPathPlanner() {

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // gives the current robot location
                this::seedFieldRelative, // takes the location and makes it the pose
                this::getCurrentChassisSpeeds, // makes a command that applies the chassis speed
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                getHolonomicPathFollowerConfig(),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // determines the red or blue team
                this);

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOveride);
    }

    public Optional<Rotation2d> getRotationTargetOveride() {

        // supply a Rotation2d in this method for the new heading of the robot when has
        // a note in shooter or has a target when empty
        return Optional.of(new Rotation2d(180));
    }

    public ChassisSpeeds getCurrentChassisSpeeds() { // creates getCurrentChassisSpeeds
        return m_kinematics.toChassisSpeeds(getState().ModuleStates); // gets the current speed
    }

    public static double getExponential(double input) {

        if (Math.abs(input) < DriveTrainConstants.kDeadBand) {
            return 0;
        }

        double sign = Math.signum(input);
        double v = Math.abs(input);

        double a = DriveTrainConstants.kWeight * Math.pow(v, DriveTrainConstants.kExponent)
                + (1 - DriveTrainConstants.kWeight) * v;
        double b = DriveTrainConstants.kWeight * Math.pow(DriveTrainConstants.kDeadBand, DriveTrainConstants.kExponent)
                + (1 - DriveTrainConstants.kWeight) * DriveTrainConstants.kDeadBand;
        v = (a - 1 * b) / (1 - b);

        rateLimeter.calculate(v);

        v *= sign;

        if (driverStationAlliance()) {
            v *= -1;
        }

        SmartDashboard.putNumber("Joystick Exponential value", v);
        return v;
    }

    public static double getRotationalExponential(double input) {

        if (Math.abs(input) < DriveTrainConstants.kDeadBand) {
            return 0;
        }

        double sign = Math.signum(input);
        double v = Math.abs(input);

        double a = DriveTrainConstants.kWeight * Math.pow(v, DriveTrainConstants.kExponent)
                + (1 - DriveTrainConstants.kWeight) * v;
        double b = DriveTrainConstants.kWeight * Math.pow(DriveTrainConstants.kDeadBand, DriveTrainConstants.kExponent)
                + (1 - DriveTrainConstants.kWeight) * DriveTrainConstants.kDeadBand;
        v = (a - 1 * b) / (1 - b);

        v *= sign;

        return v;
    }

    public Command ampPathCommand() {

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {

            if (alliance.get() == DriverStation.Alliance.Blue) {
                infrontAmpPos = FieldConstants.kInfrontBluePos;
                ampPos = FieldConstants.kBlueAmpScorePose;
            } else {
                infrontAmpPos = FieldConstants.kInfrontRedPos;
                ampPos = FieldConstants.kRedAmp;
            }
        }

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                // getState().Pose,
                infrontAmpPos,
                ampPos

        );

        PathPlannerPath newpath = new PathPlannerPath(
                bezierPoints,
                DriveTrainConstants.kPathConstraints,
                new GoalEndState(0.0, Rotation2d.fromDegrees(90)));

        return new FollowPathHolonomic(
                newpath,
                () -> this.getState().Pose, // supplies the robot pose
                this::getCurrentChassisSpeeds, // supplies the current chassis speed
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // will drive robot using given chassis
                                                                             // speeds
                getHolonomicPathFollowerConfig(), //
                () -> false, // flips path for opposite side
                this // requiering this drivetrain
        );

    }

    public Command pathfinding(String pathName) {

        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                DriveTrainConstants.kPathConstraints,
                0.25);

        return pathfindingCommand;
    }

    public Command ClimberPathFinding() {

        PathPlannerPath climb = PathPlannerPath.fromPathFile("Auto Climb Path");

        Command pathFindingCommand = AutoBuilder.pathfindThenFollowPath(
                climb,
                DriveTrainConstants.kPathConstraints,
                0.25);

        return pathFindingCommand;
    }

    public static boolean driverStationAlliance() {
        var alliance = DriverStation.getAlliance(); // creates a variable called alliance
        if (alliance.isPresent()) { // checks if alliance variable has been set to anything
            return alliance.get() == DriverStation.Alliance.Red; // checks if the alliance color is red
        }
        return false; // returns false if color is not red or if no color
    }

    private void setHeadingPID() {
        driveAngle.HeadingController.setPID(0, 0, 0);
    }

    // command to use when driving at a note or speaker
    public Command driveWithHeading(double xVelocity, double yVelocity, Rotation2d heading) {
        return applyRequest(() -> driveAngle.withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withTargetDirection(heading));
    }

    // return the position of each drive
    public double[] getWheelPosition() {
        double[] wheelPositionsRads = new double[4];

        for (int i = 0; i < 4; i++) {
            wheelPositionsRads[i] = getModule(i).getDriveMotor().getPosition().getValueAsDouble();

            // may need to add a line hear to convert each value to radians of rotoation of
            // the wheel
            // need to check this

            // this may help with the conversion of units not sure if it is right
            wheelPositionsRads[i] = Units.Radians.convertFrom(wheelPositionsRads[i], Units.Revolutions);

        }

        return wheelPositionsRads;
    }
}
