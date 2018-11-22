package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auton.Kinematic;
import frc.robot.subsystems.Drivetrain;
import frc.util.CheesyDriveHelper;
import frc.util.kinematics.RobotPos;
import frc.util.logging.CSVLogger;
import frc.util.logging.Loggable;

public class Robot extends IterativeRobot{;

    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    public static RobotPos mMasterPos = new RobotPos( 0, 0, 0 );
    private Loggable mPositionLog;
    private static final double kLocalizerTimestep = 0.05;

    private Notifier mLocalizer;

    @Override
    public void autonomousInit() {
        mLocalizer.startPeriodic(kLocalizerTimestep);
    }

    @Override
    public void robotInit(){
        Drivetrain.getInstance().zeroSensor();
        setupLogger();
        mLocalizer = new Notifier(new Kinematic(mMasterPos, kLocalizerTimestep));
    }
    
    @Override
    public void teleopInit(){
        mLocalizer = new Notifier(new Kinematic(mMasterPos, kLocalizerTimestep));
        mLocalizer.startPeriodic(kLocalizerTimestep);
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
        mPositionLog.log();
    }

    @Override
    public void disabledInit() {
        CSVLogger.logCSV("logs/DriveLog", Drivetrain.driveLogger.get());
        CSVLogger.logCSV("logs/PosLog", mPositionLog.get());

        Drivetrain.driveLogger.clearLog();
        mPositionLog.clearLog();

        mLocalizer.stop();
    }

    @Override
    public void testInit(){
    }

    private void setupLogger(){
        mPositionLog = new Loggable(){
            @Override
            protected LogObject[] collectData() {
                return new LogObject[]{
                    new LogObject("Time", Timer.getFPGATimestamp()),
                    new LogObject("XPos", mMasterPos.getX()),
                    new LogObject("YPos", mMasterPos.getY()),
                };
            }
        };
    }
}