package frc.robot;

import java.io.File;
import java.util.LinkedHashMap;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auton.MoveCommand;
import frc.robot.auton.PathLoader;
import frc.robot.auton.Ramsete;
import frc.robot.subsystems.Drivetrain;
import frc.util.CheesyDriveHelper;
import frc.util.kinematics.pos.RobotPos;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class Robot extends IterativeRobot{;

    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    public static final double kLocalizerTimestep = 0.005;
    public static final double kLoggingTimestep = 0.1;
    public static final double kRamseteTimestep = 0.01;

    public static LinkedHashMap<String, Trajectory> autonPaths;

    Command autonCommand;

    // private UsbCamera driveCam;

    @Override
    public void robotInit(){

        // driveCam = CameraServer.getInstance().startAutomaticCapture();
		// driveCam.setResolution(320, 240);
        // driveCam.setFPS(12);

        Drivetrain.getInstance().zeroSensor();

        //verify both Odometery and Ramsete are constructed
        // Odometery.getInstance();
        Ramsete.getInstance();
        PeriodicLogger.getInstance();

        //Add Loggable to Periodic Logger
        PeriodicLogger.addLoggable(Drivetrain.getInstance());
        PeriodicLogger.addLoggable(Ramsete.getInstance());

        CameraServer.getInstance().startAutomaticCapture();

        autonPaths = PathLoader.loadPaths();

        autonCommand = new MoveCommand(autonPaths.get("LeftCurve"));

        PeriodicLogger.getInstance().start();
    }

    @Override
    public void autonomousInit() {
        intoEnabled();
        Drivetrain.getInstance().zeroSensor();

        Drivetrain.getInstance().startOdometery(kLocalizerTimestep);
        Ramsete.getInstance().start();

        autonCommand.start();
    }

    @Override
    public void autonomousPeriodic() {
        //no-op
    }
    
    @Override
    public void teleopInit(){
        intoEnabled();
    }

    @Override
    public void teleopPeriodic() {
        Drivetrain.getInstance().setRawSpeed(
            mCheesyDriveHelper.cheesyDrive(
            OI.getInstance().getForward(),
            OI.getInstance().getLeft(),
            OI.getInstance().getQuickTurn()
            , true)
        );
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledInit() {
        PeriodicLogger.getInstance().stop();
        PeriodicLogger.allToCSV();
        PeriodicLogger.clearAll();

        Drivetrain.getInstance().stopOdometery();
        Ramsete.getInstance().stop();
        System.out.println(Drivetrain.getInstance().getRobotX() + "\t\t\t" + Drivetrain.getInstance().getRobotY());
    }

    public void intoEnabled(){
        PeriodicLogger.getInstance().start();
    }
}