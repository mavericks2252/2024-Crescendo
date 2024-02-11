package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    Pose2d ampPos = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d infrontAmpPos = new Pose2d(0, 0, new Rotation2d(0));

   


    
       
    

    public final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds(); //creates a swerve request called auto request

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configPathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configPathPlanner();
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

    public HolonomicPathFollowerConfig getHolonomicPathFollowerConfig(){
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
            //finds the radius by taking the larger of the locations
           } 

            return new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0), //constants for the translation controller
                                                    new PIDConstants(10, 0, 0), //constants for the rotation controller
                                                    TunerConstants.kSpeedAt12VoltsMps, //sets the max speed 
                                                    driveBaseRadius,
                                                    new ReplanningConfig());
        
    }


    //Configure auto builder
    private void configPathPlanner(){
        

        AutoBuilder.configureHolonomic(
                ()->this.getState().Pose, //gives the current robot location
                this::seedFieldRelative,  //takes the location and makes it the pose
                this::getCurrentChassisSpeeds,  //makes a command that applies the chassis speed
                (speeds)->this.setControl(autoRequest.withSpeeds(speeds)),
                getHolonomicPathFollowerConfig(),
                 ()->false, //determines the red or blue team
                 this);
    }

    public ChassisSpeeds getCurrentChassisSpeeds(){ //creates getCurrentChassisSpeeds
        return m_kinematics.toChassisSpeeds(getState().ModuleStates); //gets the current speed
    }

    public static double getExponential(double input){

        if (Math.abs(input) < DriveTrainConstants.kDeadBand) {
            return 0;
        }

        double sign = Math.signum(input);
        double v = Math.abs(input);
        
        double a = DriveTrainConstants.kWeight * Math.pow(v, DriveTrainConstants.kExponent) + (1 - DriveTrainConstants.kWeight) * v;
        double b = DriveTrainConstants.kWeight * Math.pow(DriveTrainConstants.kDeadBand, DriveTrainConstants.kExponent) + (1 - DriveTrainConstants.kWeight) * DriveTrainConstants.kDeadBand;
        v = (a - 1 * b) / (1 - b);

        v *= sign;
        return v;
    }

   

        
    

    public Command ampPathCommand(){


        var alliance = DriverStation.getAlliance();        
        
        if (alliance.isPresent()) {
            
            if(alliance.get() == DriverStation.Alliance.Blue){
                infrontAmpPos = FieldConstants.kInfrontBluePos;
                ampPos = FieldConstants.kBlueAmpScorePose;
            }
            else {
                infrontAmpPos = FieldConstants.kInfrontRedPos;
                ampPos = FieldConstants.kRedAmp;
            } 
        }

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            //getState().Pose,
            infrontAmpPos,
            ampPos
            
        );

        PathPlannerPath newpath = new PathPlannerPath(
            bezierPoints,
            DriveTrainConstants.kPathConstraints,
            new GoalEndState(0.0, Rotation2d.fromDegrees(90))
            );
        
        return new FollowPathHolonomic(
            newpath, 
            ()-> this.getState().Pose, //supplies the robot pose
            this::getCurrentChassisSpeeds, //supplies the current chassis speed
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), //will drive robot using given chassis speeds
            getHolonomicPathFollowerConfig(), //
            ()->false, //flips path for opposite side
            this //requiering this drivetrain
            );



    }


   
}
