package frc.robot.auton;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.util.kinematics.OdometeryUtil;
import frc.robot.Robot;

public class Odometery extends OdometeryUtil implements Runnable{

    private Notifier mNotifier;
    private final double kTimestep;

    private static final double kWheelRadius = Constants.kWheelRadius;
    private static final double kWheelBase = Constants.kWheelBase;

    private static Odometery instance;
    public static Odometery getInstance(){
        if (instance == null)
            instance = new Odometery(Robot.kLocalizerTimestep);
        return instance;
    }

    public Odometery(double timestep){
        super(kWheelRadius, kWheelBase, timestep);
        mNotifier = new Notifier(this);
        kTimestep = timestep;
    }

    public void start(){
        mNotifier.startPeriodic(kTimestep);
    }

    public void stop(){
        mNotifier.stop();
    }

    @Override
    public void run() {
        Robot.mMasterPos.add(this.getDeltas());
        Robot.mPositionLog.log();
    }

    @Override
    protected double getLeftWheelVelocity() {
        return Drivetrain.getInstance().getLeftSensorVelocity();
    }

    @Override
    protected double getRightWheelVelocity() {
        return Drivetrain.getInstance().getRightSensorVelocity();
    }

    @Override
    protected double getHeading(){
        return Drivetrain.getInstance().getGyroAngle();
    }
}