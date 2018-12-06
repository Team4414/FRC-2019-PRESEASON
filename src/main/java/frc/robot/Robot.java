package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import frc.robot.auton.Odometery;
import frc.robot.auton.Ramsete;
import frc.robot.subsystems.Drivetrain;
import frc.util.CheesyDriveHelper;
import frc.util.kinematics.pos.Pose2d;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class Robot extends IterativeRobot{;

    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    public static final double kLocalizerTimestep = 0.01;
    public static final double kLoggingTimestep = 0.1;
    public static final double kRamseteTimestep = 0.01;

    private Trajectory autonTraj;

    @Override
    public void robotInit(){
        Drivetrain.getInstance().zeroSensor();

        //verify both Odometery and Ramsete are constructed
        Odometery.getInstance();
        Ramsete.getInstance();
        PeriodicLogger.getInstance();

        //Add Loggable to Periodic Logger
        PeriodicLogger.addLoggable(Drivetrain.getInstance());
        PeriodicLogger.addLoggable(Ramsete.getInstance());

        CameraServer.getInstance().startAutomaticCapture();

        autonTraj = getTrajFromFile("/home/lvuser/autoPaths/TestPath_left_detailed.csv");

        PeriodicLogger.getInstance().start();
    }

    @Override
    public void autonomousInit() {
        intoEnabled();
        Drivetrain.getInstance().zeroSensor();

        Drivetrain.masterPos = new Pose2d();

        Odometery.getInstance().start();
        Ramsete.getInstance().start();

        Ramsete.getInstance().trackPath(autonTraj);
    }
    
    @Override
    public void teleopInit(){
        intoEnabled();
        Odometery.getInstance().start();
    }

    @Override
    public void teleopPeriodic() {
        // Drivetrain.getInstance().setRawSpeed(
        //     mCheesyDriveHelper.cheesyDrive(
        //     OI.getInstance().getForward(),
        //     OI.getInstance().getLeft(),
        //     OI.getInstance().getQuickTurn()
        //     , true)
        // );
        Drivetrain.getInstance().setVelocity(6, 6);

        
        // Drivetrain.getInstance().setRawSpeed(1, 1);
        // System.out.println(OI.getInstance().getForward() + "\t|\t" + OI.getInstance().getLeft());
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledInit() {
        PeriodicLogger.getInstance().stop();
        PeriodicLogger.allToCSV();
        PeriodicLogger.clearAll();

        Odometery.getInstance().stop();
        Ramsete.getInstance().stop();
    }


    private Trajectory getTrajFromFile(String file){
        return Pathfinder.readFromCSV(new File(file));
    //     Waypoint[] points = new Waypoint[] {

    //         new Waypoint(0, 0, Pathfinder.d2r(0)), // Waypoint @ x=0, y=0, exit angle=0 radians						// angle=-45 degrees
    //         new Waypoint(4, 4, Pathfinder.d2r(0)), // Waypoint @ x=-2, y=-2, exit angle=0 radians				// angle=-45 degrees
            
    //     };

    //     Trajectory.Config config;
    //     Trajectory trajectory;

        
    //     config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
    //             kRamseteTimestep, 4, 4, 10000);
    //     trajectory = Pathfinder.generate(points, config);

    //     System.out.println(trajectory.length());
    //     return trajectory;
    }

    public void intoEnabled(){
        PeriodicLogger.getInstance().start();
    }
}