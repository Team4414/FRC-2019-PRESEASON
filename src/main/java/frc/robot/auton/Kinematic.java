package frc.robot.auton;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.util.kinematics.ForwardKinematics;
import frc.util.kinematics.RobotPos;

public class Kinematic implements Runnable{

    private static final double kWheelRadius = 1;
    private static final double kWheelBase = 1;

    private ForwardKinematics kine;

    private class DriveKinematics extends ForwardKinematics{

        //DriveKinematics is an implementation of ForwardKinematics

        public DriveKinematics(RobotPos initialPos, double timestep){
            super(kWheelRadius, kWheelBase, timestep, initialPos.getX(), initialPos.getY(), initialPos.getHeading());
        }

        @Override
        protected double getLeftWheelVelocity() {
            return Drivetrain.getInstance().getLeftSensorVelocity();
        }

        @Override
        protected double getRightWheelVelocity() {
            return Drivetrain.getInstance().getRightSensorVelocity();
        }

    }

    public Kinematic(RobotPos initialPos, double timestep){
        kine = new DriveKinematics(initialPos, timestep);
    }

    @Override
    public void run() {
        kine.update();

        Robot.mMasterPos = new RobotPos(kine.getXPosition(), kine.getXPosition(), kine.getHeading()); 
    }
}