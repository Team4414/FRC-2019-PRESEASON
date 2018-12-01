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
            getLeftWheelVelocity() * kTimestep,
            getRightWheelVelocity() *kTimestep, 
            getHeading().getRadians()));
    }

    protected double getLeftWheelVelocity() {
        return Drivetrain.getInstance().getLeftSensorVelocity();
    }

    protected double getRightWheelVelocity() {
        return Drivetrain.getInstance().getRightSensorVelocity();
    }

    protected Rotation2d getHeading(){
        return Drivetrain.getInstance().getGyroAngle();
    }
}