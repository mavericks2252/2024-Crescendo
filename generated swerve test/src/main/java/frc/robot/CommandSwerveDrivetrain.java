package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;


    
       
    

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

    //Configure auto builder
    private void configPathPlanner(){
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
            //finds the radius by taking the larger of the locations
        }

        AutoBuilder.configureHolonomic(
                ()->this.getState().Pose, //gives the current robot location
                this::seedFieldRelative,  //takes the location and makes it the pose
                this::getCurrentChassisSpeeds, 
                (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), //makes a command that applies the chassis speed
                 new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0), //constants for the translation controller
                                                    new PIDConstants(10, 0, 0), //constants for the rotation controller
                                                    TunerConstants.kSpeedAt12VoltsMps, //sets the max speed 
                                                    driveBaseRadius,
                                                    new ReplanningConfig()),
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
   
}
