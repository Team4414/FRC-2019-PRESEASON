package frc.robot.auton;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.util.kinematics.pos.Rotation2d;
import frc.util.kinematics.pos.Twist2d;

public class Odometery implements Runnable{

    private Notifier mNotifier;
    private final double kTimestep;

    private static Odometery instance;

    private double lastLeftPos  = 0;
    private double lastRightPos = 0;
    public static Odometery getInstance(){
        if (instance == null)
            instance = new Odometery(Robot.kLocalizerTimestep);
        return instance;
    }

    public Odometery(double timestep){
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
        Drivetrain.getInstance();
        Drivetrain.masterPos = Drivetrain.masterPos.transformBy(Twist2d.fromWheels(
            getLeftWheelVelocity(),
            getRightWheelVelocity(), 
            getHeading().getRadians()));
    }

    protected double getLeftWheelVelocity() {
        // return Drivetrain.getInstance().getLeftSensorVelocity();
        double returnMe = (Drivetrain.getInstance().getLeftSensorPosition() - lastLeftPos);
        lastLeftPos = Drivetrain.getInstance().getLeftSensorPosition();
        return returnMe;
    }

    protected double getRightWheelVelocity() {
        // return Drivetrain.getInstance().getRightSensorVelocity();
        double returnMe = (Drivetrain.getInstance().getRightSensorPosition() - lastRightPos);
        lastRightPos = Drivetrain.getInstance().getRightSensorPosition();
        return returnMe;
    }

    protected Rotation2d getHeading(){
        return Drivetrain.getInstance().getGyroAngle();
    }
}