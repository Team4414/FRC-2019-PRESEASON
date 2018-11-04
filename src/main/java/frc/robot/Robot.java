package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.auton.Kinematic;
import frc.robot.subsystems.Drivetrain;
import frc.util.CheesyDriveHelper;
import frc.util.kinematics.RobotPos;
import frc.util.logging.CSVLogger;

public class Robot extends IterativeRobot{

    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    public static RobotPos mMasterPos = new RobotPos( 0, 0, 0 );
    private static final double kLocalizerTimestep = 0.05;

    private Notifier mLocalizer;

    @Override
    public void autonomousInit() {
        mLocalizer = new Notifier(new Kinematic(mMasterPos, kLocalizerTimestep));
        mLocalizer.startPeriodic(kLocalizerTimestep);
    }

    @Override
    public void robotInit(){
        Drivetrain.getInstance();
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
        CSVLogger.logCSV("logs/DriveLog", Drivetrain.driveLogger.get());
    }

}