package frc.robot.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.util.auton.RamseteUtil.Status;
import jaci.pathfinder.Trajectory;

public class MoveCommand extends Command{

    private Trajectory mPath;

    public MoveCommand(Trajectory path){
        mPath = path;
    }

    @Override
    protected void initialize() {
        if (!Ramsete.isRunning()){
            System.out.println("!!!!!!!!!! Attempted to start movement without starting Ramsete Controller !!!!!!!!!!");
        }else{
            Ramsete.getInstance().trackPath(mPath);
        }
        System.out.println("Starting Move");
    }

    @Override
    protected boolean isFinished() {
        System.out.println("Status: " + Ramsete.getStatus());
        return Ramsete.getStatus() == Status.STANDBY;
    }

    @Override
    protected void end() {
        System.out.println("Move Finished");
        Drivetrain.getInstance().setVelocity(0, 0);
    }

    @Override
    protected void interrupted() {
        System.out.println("Move Interrupted");
    }

}