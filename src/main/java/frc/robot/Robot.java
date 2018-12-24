package frc.robot;

import java.util.LinkedHashMap;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.auton.MoveCommand;
import frc.robot.auton.PathLoader;
import frc.robot.auton.Ramsete;
import frc.robot.subsystems.Drivetrain;
import frc.util.CheesyDriveHelper;
import jaci.pathfinder.Trajectory;

public class Robot extends IterativeRobot{

    //Teleoperated Mode Controller;
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();


    //THREAD TIMESTEPS (in seconds):
    public static final double kLocalizerTimestep = 0.005;
    public static final double kLoggingTimestep = 0.1;
    public static final double kRamseteTimestep = 0.01;

    //Map of all autonomous paths
    public static LinkedHashMap<String, Trajectory> autonPaths;

    //The selected autonomous command to run.
    Command autonCommand;

    //Drive Camera
    // private UsbCamera driveCam;

    @Override
    public void robotInit(){

        //Drive Camera Initialization:
        // driveCam = CameraServer.getInstance().startAutomaticCapture();
		// driveCam.setResolution(320, 240);
        // driveCam.setFPS(12);
        // CameraServer.getInstance().startAutomaticCapture();

        //Zero All Subsystems:
        Drivetrain.getInstance().zeroSensor();

        //verify both Odometery and Ramsete are constructed
        Ramsete.getInstance();
        PeriodicLogger.getInstance();


        //Import all autonomous paths from filesystem (time intensive)
        autonPaths = PathLoader.loadPaths();

        //select the autonomous command
        //in competition this will likely be done in autonomousInit()
        autonCommand = new MoveCommand(autonPaths.get("DriveLeftCurve"));


        //Add Loggables to Periodic Logger
        PeriodicLogger.addLoggable(Drivetrain.getInstance());
        PeriodicLogger.addLoggable(Ramsete.getInstance());

        //Start the Periodic Logger.
        PeriodicLogger.getInstance().start();
    }

    @Override
    public void autonomousInit() {
        intoEnabled();

        //Zero relevant subsystems:
        Drivetrain.getInstance().zeroSensor();

        //Start an Odometery thread to begin keeping track of the vehicle position
        Drivetrain.getInstance().startOdometery(kLocalizerTimestep);
        
        //Start the autonomous controller
        Ramsete.getInstance().start();

        //Start the selected autonomous command.
        autonCommand.start();
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }
    
    @Override
    public void teleopInit(){
        intoEnabled();
    }

    @Override
    public void teleopPeriodic() {

        //Update Inputs and Outputs
        Drivetrain.getInstance().setRawSpeed(
            mCheesyDriveHelper.cheesyDrive(
            OI.getInstance().getForward(),
            OI.getInstance().getLeft(),
            OI.getInstance().getQuickTurn()
            , true)
        );
    }

    @Override
    public void disabledInit() {
        PeriodicLogger.getInstance().stop();
        PeriodicLogger.allToCSV();
        PeriodicLogger.clearAll();

        Drivetrain.getInstance().stopOdometery();
        Ramsete.getInstance().stop();
        System.out.println(Drivetrain.getInstance().getRobotPos().getX() + "\t\t\t" + Drivetrain.getInstance().getRobotPos().getY());
    }

    public void intoEnabled(){
        PeriodicLogger.getInstance().start();
    }
}