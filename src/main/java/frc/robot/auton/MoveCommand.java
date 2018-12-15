package frc.robot.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.util.auton.RamseteUtil.Status;
import jaci.pathfinder.Trajectory;

public class MoveCommand extends Command{

    private Trajectory mPath;

    public MoveCommand(Trajectory path){
        mPath = path;
    }

    @Override
    protected void initialize() {
        if (Ramsete.getStatus() == Status.STOPPED){
            System.out.println("!!!!!!!!!! Attempted to start movement without starting Ramsete Controller !!!!!!!!!!");
        }else{
            Ramsete.getInstance().trackPath(mPath);
        }
    }

    @Override
    protected boolean isFinished() {
        return Ramsete.getStatus() == Status.STANDBY;
    }

    @Override
    protected void interrupted() {
        Ramsete.getInstance().setStatus(Status.STANDBY, true);
    }

}