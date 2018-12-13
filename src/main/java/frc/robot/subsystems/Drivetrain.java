package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.kinematics.pos.Pose2d;
import frc.util.kinematics.pos.Rotation2d;
import frc.util.logging.ILoggable;
import frc.util.logging.Loggable;
import frc.util.talon.TalonSRXFactory;

public class Drivetrain extends Subsystem implements ILoggable{

    public static Pose2d masterPos = new Pose2d();

    private static final int kPIDidx = 0;
    private static final int kCTRETimeout = 0; //no error reporting

    private static Drivetrain instance;
    public static Drivetrain getInstance(){
        if(instance == null)
            instance = new Drivetrain();
        return instance;
    }

    private TalonSRX mLeftMaster, mRightMaster;

    private VictorSPX mLeftSlaveA, mLeftSlaveB, mRightSlaveA, mRightSlaveB;

    private PigeonIMU mGyro;

    private int mLeftZeroOffset = 0;
    private int mRightZeroOffset = 0;
    private double mGyroOffset = 0;

    private double kP = 0.4;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0.17;

    private double kFriction = 0.1;

    private Drivetrain(){
        mLeftMaster = TalonSRXFactory.createDefaultTalon(RobotMap.DrivetrainMap.kLeftMaster);
        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kLeftSlaveA, mLeftMaster);
        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kLeftSlaveB, mLeftMaster);

        mRightMaster = TalonSRXFactory.createDefaultTalon(RobotMap.DrivetrainMap.kRightMaster);
        mRightSlaveA = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kRightSlaveA, mRightMaster);
        mRightSlaveB = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kRightSlaveB, mRightMaster);

        mLeftMaster.configOpenloopRamp(0.01, kCTRETimeout);
        mRightMaster.configOpenloopRamp(0.01, kCTRETimeout);

        mGyro = new PigeonIMU(0);

        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDidx, kCTRETimeout);
        mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDidx,kCTRETimeout);

        mLeftMaster.config_kP(0, kP, kCTRETimeout);
        mLeftMaster.config_kI(0, kI, kCTRETimeout);
        mLeftMaster.config_kD(0, kD, kCTRETimeout);
        mLeftMaster.config_kF(0, kF, kCTRETimeout);

        mRightMaster.config_kP(0, kP, kCTRETimeout);
        mRightMaster.config_kI(0, kI, kCTRETimeout);
        mRightMaster.config_kD(0, kP, kCTRETimeout);
        mRightMaster.config_kF(0, kF, kCTRETimeout);

        mLeftMaster.setSensorPhase(true);
        mRightMaster.setSensorPhase(true);

        mRightMaster.setInverted(true);
        mRightSlaveA.setInverted(true);
        mRightSlaveB.setInverted(true);

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

    public void setVelocity(double left, double right){
        // setRawSpeed(
        //     left/12, right/12
        // );

        System.out.println(mLeftMaster.getClosedLoopError(0) * Constants.kNativeU2FPS);

        mLeftMaster.set(ControlMode.Velocity, left * Constants.kFPS2NativeU, DemandType.ArbitraryFeedForward , kFriction);
        mRightMaster.set(ControlMode.Velocity, right * Constants.kFPS2NativeU, DemandType.ArbitraryFeedForward, kFriction);
    }

    public void zeroSensor(){
        mLeftZeroOffset = mLeftMaster.getSelectedSensorPosition(kPIDidx);
        mRightZeroOffset = mRightMaster.getSelectedSensorPosition(kPIDidx);
        mGyroOffset = mGyro.getFusedHeading();
    }

    public Rotation2d getGyroAngle(){
        return Rotation2d.fromDegrees(mGyro.getFusedHeading() - mGyroOffset);
    }

    /**
     * @return Left Sensor Position in Feet
     */
    public double getLeftSensorPosition(){
        return Constants.kTicks2Feet * (mLeftMaster.getSelectedSensorPosition(kPIDidx) - mLeftZeroOffset);
    }

    /**
     * @return Right Sensor Position in Feet
     */
    public double getRightSensorPosition(){
        return Constants.kTicks2Feet * (mRightMaster.getSelectedSensorPosition(kPIDidx) - mRightZeroOffset);
    }

    /**
     * @return Left Sensor Velocity in Feet per Second
     */
    public double getLeftSensorVelocity(){
        return Constants.kNativeU2FPS * mLeftMaster.getSelectedSensorVelocity(kPIDidx);
    }

    /**
     * @return Left Sensor Velocity in Feet per Second
     */
    public double getRightSensorVelocity(){
        return Constants.kNativeU2FPS * mRightMaster.getSelectedSensorVelocity(kPIDidx);
    }

    @Override
    public Loggable setupLogger(){
        return new Loggable("DriveLog"){
            @Override
            protected LogObject[] collectData() {
                return new LogObject[]{
                    new LogObject("LeftVel",  getLeftSensorVelocity()),
                    new LogObject("RightVel", getRightSensorVelocity()),

                    new LogObject("Type", "R"),
                    new LogObject("XPos", masterPos.getTranslation().x()),
                    new LogObject("YPos", masterPos.getTranslation().y()),
                    new LogObject("Heading", masterPos.getRotation().getRadians())
                };
            }
        };
    }
}