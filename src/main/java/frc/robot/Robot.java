package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import frc.robot.subsystems.Drivetrain;
import frc.util.CheesyDriveHelper;
import frc.util.logging.CSVLogger;

public class Robot extends IterativeRobot{

    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

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
    }

}