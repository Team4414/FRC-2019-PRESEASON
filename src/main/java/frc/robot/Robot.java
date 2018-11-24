package frc.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auton.AutonController;
import frc.robot.auton.Odometery;
import frc.robot.auton.Ramsete;
import frc.robot.subsystems.Drivetrain;
import frc.util.CheesyDriveHelper;
import frc.util.kinematics.RobotPos;
import frc.util.logging.CSVLogger;
import frc.util.logging.Loggable;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class Robot extends IterativeRobot{;

    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    public static RobotPos mMasterPos = new RobotPos( 0, 0, 0 );
    public static Loggable mPositionLog;
    public static final double kLocalizerTimestep = 0.01;
    public static final double kLoggingTimestep = 0.1;
    public static final double kRamseteTimestep = 0.01;

    private Notifier mLogger;

    private Trajectory autonTraj;

    @Override
    public void robotInit(){
        Drivetrain.getInstance().zeroSensor();

        setupLogger();

        //verify both Odometery and Ramsete are constructed
        Odometery.getInstance();
        Ramsete.getInstance();
        
        mLogger = new Notifier( new Runnable(){
        
            @Override
            public void run() {
                CSVLogger.logCSV("logs/DriveLog", Drivetrain.driveLogger.get());
                CSVLogger.logCSV("logs/PosLog", mPositionLog.get());
                CSVLogger.logCSV("logs/PathLog", Ramsete.getInstance().mPathLogger.get());
            }
        });

        mLogger.startPeriodic(kLoggingTimestep);

        CameraServer.getInstance().startAutomaticCapture();

        autonTraj = getTrajFromFile();
    }

    @Override
    public void autonomousInit() {

        mMasterPos = new RobotPos(0, 0, 0);

        Odometery.getInstance().start();
        Ramsete.getInstance().start();

        Ramsete.getInstance().trackPath(autonTraj);
    }
    
    @Override
    public void teleopInit(){
        Odometery.getInstance().start();
    }

    @Override
    public void teleopPeriodic() {
        Drivetrain.getInstance().setRawSpeed(
            mCheesyDriveHelper.cheesyDrive(
            OI.getInstance().getForward(),
            OI.getInstance().getLeft(),
            OI.getInstance().getQuickTurn()
            , false)
        );
    }

    @Override
    public void robotPeriodic() {
        Drivetrain.driveLogger.log();
    }

    @Override
    public void disabledInit() {

        mLogger.stop();

        Drivetrain.driveLogger.clearLog();
        mPositionLog.clearLog();

        Odometery.getInstance().stop();
        Ramsete.getInstance().stop();
    }

    private void setupLogger(){
        mPositionLog = new Loggable(){
            @Override
            protected LogObject[] collectData() {
                return new LogObject[]{
                    new LogObject("Time", Timer.getFPGATimestamp()),
                    new LogObject("Type", "r"),
                    new LogObject("X Pos", mMasterPos.getX()),
                    new LogObject("Y Pos", mMasterPos.getY()),
                    new LogObject("Heading", mMasterPos.getHeading())
                };
            }
        };
    }


    private Trajectory getTrajFromFile(){
        // return Pathfinder.readFromCSV(new File(file));
        Waypoint[] points = new Waypoint[] {

            new Waypoint(0, 0, Pathfinder.d2r(0)), // Waypoint @ x=0, y=0, exit angle=0 radians						// angle=-45 degrees
            new Waypoint(4, 4, Pathfinder.d2r(0)), // Waypoint @ x=-2, y=-2, exit angle=0 radians				// angle=-45 degrees
            
        };

        Trajectory.Config config;
        Trajectory trajectory;

        
        config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
                kRamseteTimestep, 4, 4, 10000);
        trajectory = Pathfinder.generate(points, config);

        System.out.println(trajectory.length());
        return trajectory;
    }
}