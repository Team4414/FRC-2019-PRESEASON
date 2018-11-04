package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.util.logging.Loggable;
import frc.util.talon.TalonSRXFactory;

public class Drivetrain extends Subsystem{

    private static Drivetrain instance;
    public static Drivetrain getInstance(){
        if(instance == null)
            instance = new Drivetrain();
        return instance;
    }

    public static Loggable driveLogger;

    private TalonSRX mLeftMaster, mRightMaster;

    private VictorSPX mLeftSlaveA, mLeftSlaveB, mRightSlaveA, mRightSlaveB;

    private int mLeftZeroOffset = 0;
    private int mRightZeroOffset = 0;

    private Drivetrain(){
        mLeftMaster = TalonSRXFactory.createDefaultTalon(RobotMap.DrivetrainMap.kLeftMaster);
        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kLeftSlaveA, mLeftMaster);
        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kLeftSlaveB, mLeftMaster);

        mRightMaster = TalonSRXFactory.createDefaultTalon(RobotMap.DrivetrainMap.kRightMaster);
        mRightSlaveA = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kRightSlaveA, mRightMaster);
        mRightSlaveB = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kRightSlaveB, mRightMaster);

        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        setupLogger();
    }

    @Override
    protected void initDefaultCommand(){};

    public void setRawSpeed(double left, double right){
        mLeftMaster.set(ControlMode.PercentOutput, left);
        mRightMaster.set(ControlMode.PercentOutput, right);
    }

    public void setRawSpeed(double[] speeds){
        setRawSpeed(speeds[0], speeds[1]);
    }

    public void zeroSensor(){
        mLeftZeroOffset = mLeftMaster.getSelectedSensorPosition();
        mRightZeroOffset = mRightMaster.getSelectedSensorPosition();
    }

    public int getLeftSensorPosition(){
        return (mLeftMaster.getSelectedSensorPosition() - mLeftZeroOffset);
    }

    public int getRightSensorPosition(){
        return (mRightMaster.getSelectedSensorPosition() - mRightZeroOffset);
    }

    public int getLeftSensorVelocity(){
        return mLeftMaster.getSelectedSensorVelocity();
    }

    public int getRightSensorVelocity(){
        return mRightMaster.getSelectedSensorVelocity();
    }

    private void setupLogger(){
        driveLogger = new Loggable(){
            @Override
            protected LogObject[] collectData() {
                return new LogObject[]{
                    new LogObject("LeftMasterCurrent", mLeftMaster.getOutputCurrent()),
                    new LogObject("RightMasterCurrent", mRightMaster.getOutputCurrent()),
                };
            }
        };
    }
}