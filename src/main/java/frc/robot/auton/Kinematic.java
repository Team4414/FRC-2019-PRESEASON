package frc.robot.auton;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.util.kinematics.OdometeryUtil;
import frc.util.kinematics.RobotPos;

public class Kinematic implements Runnable{

    private static final double kWheelRadius = Constants.kWheelRadius;
    private static final double kWheelBase = Constants.kWheelBase;

    private OdometeryUtil kine;

    private class DriveKinematics extends OdometeryUtil{

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

        @Override
        protected double getInputHeading(){
            return Drivetrain.getInstance().getGyroAngle();
        }

    }

    public Kinematic(RobotPos initialPos, double timestep){
        kine = new DriveKinematics(initialPos, timestep);
    }

    @Override
    public void run() {
        kine.update();
        // System.out.println("Kine Running");

        Robot.mMasterPos = new RobotPos(kine.getXPosition(), kine.getYPosition(), Drivetrain.getInstance().getGyroAngle());
        Robot.mPositionLog.log();
    }

    public void updatePos(RobotPos pos){
        kine.setPos(pos);
    }
}