package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.kinematics.pos.RobotPos;
import frc.util.logging.ILoggable;
import frc.util.logging.Loggable;
import frc.util.talon.TalonSRXFactory;
import jaci.pathfinder.Pathfinder;

public class Drivetrain extends Subsystem implements ILoggable{

    private static final int kPIDidx = 0;
    private static final int kCTRETimeout = 0; //no error reporting

    //Odometery variables:
    private double mLastPos, mCurrentPos, mDeltaPos;
    volatile double x, y, theta;
    private Notifier odometery;

    //Singleton:
    private static Drivetrain instance;
    public static Drivetrain getInstance(){
        if(instance == null)
            instance = new Drivetrain();
        return instance;
    }

    //Hardware Controllers:
    private TalonSRX mLeftMaster, mRightMaster;

    private VictorSPX mLeftSlaveA, mLeftSlaveB, mRightSlaveA, mRightSlaveB;

    private PigeonIMU mGyro;

    //Zero Offsets:
    private double mLeftZeroOffset = 0;
    private double mRightZeroOffset = 0;
    private double mGyroOffset = 0;

    //Drive Gains: 
    private double kP = 0.4;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0.17;

    private double kFriction = 0; //0.1

    private Drivetrain(){
        mLeftMaster = TalonSRXFactory.createDefaultTalon(RobotMap.DrivetrainMap.kLeftMaster);
        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kLeftSlaveA, mLeftMaster);
        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kLeftSlaveB, mLeftMaster);

        mRightMaster = TalonSRXFactory.createDefaultTalon(RobotMap.DrivetrainMap.kRightMaster);
        mRightSlaveA = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kRightSlaveA, mRightMaster);
        mRightSlaveB = TalonSRXFactory.createPermanentSlaveVictor(RobotMap.DrivetrainMap.kRightSlaveB, mRightMaster);

        mGyro = new PigeonIMU(0);

        mLeftMaster.configOpenloopRamp(0.01, kCTRETimeout);
        mRightMaster.configOpenloopRamp(0.01, kCTRETimeout);

        mLeftMaster.configVoltageCompSaturation(12, kCTRETimeout);
        mRightMaster.configVoltageCompSaturation(12, kCTRETimeout);

        mLeftMaster.enableVoltageCompensation(true);
        mRightMaster.enableVoltageCompensation(true);

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

        //Zero Odometery:
        x = 0;
        y = 0;
        theta = 0;
        
        //Set up Odometery Notifier
        odometery = new Notifier(() ->{
            mCurrentPos = (getLeftSensorPosition() + getRightSensorPosition())/2.0;
            mDeltaPos = mCurrentPos - mLastPos;
            theta = getGyroAngle();
            x +=  Math.cos(Pathfinder.d2r((theta))) * mDeltaPos;
            y +=  Math.sin(Pathfinder.d2r((theta))) * mDeltaPos;
            mLastPos = mCurrentPos;
        });
    }

    @Override
    protected void initDefaultCommand(){ /*no op*/ };

    /**
     * Set Raw Speed Method
     * 
     * @param left Percent Output
     * @param right Percent Output
     */
    public void setRawSpeed(double left, double right){
        mLeftMaster.set(ControlMode.PercentOutput, left);
        mRightMaster.set(ControlMode.PercentOutput, right);
    }

    /**
     * Set Raw Speed Method
     * 
     * @param speeds Percent Outputs
     */
    public void setRawSpeed(double[] speeds){
        setRawSpeed(speeds[0], speeds[1]);
    }

    /**
     * Set Velocity Method
     * 
     * @param left  Feet per Second
     * @param right Feet per Second
     */
    public void setVelocity(double left, double right){
        mLeftMaster.set(ControlMode.Velocity, left * Constants.kFPS2NativeU, DemandType.ArbitraryFeedForward , kFriction);
        mRightMaster.set(ControlMode.Velocity, right * Constants.kFPS2NativeU, DemandType.ArbitraryFeedForward, kFriction);
    }

    /**
     * Zero Sensor
     * 
     * <p> Zeroes all sensors (encoders + gyro) and odometery information </p>
     */
    public void zeroSensor(){
        mLeftZeroOffset = mLeftMaster.getSelectedSensorPosition(kPIDidx);
        mRightZeroOffset = mRightMaster.getSelectedSensorPosition(kPIDidx);
        mGyroOffset = mGyro.getFusedHeading();
        
        x = 0;
        y = 0;
        theta = 0;
    }

    /**
     * Start Odometery Method
     * 
     * <p> Starts tracking the robot position </p>
     * 
     * @param period timestep to update at.
     */
    public void startOdometery(double period){
        odometery.startPeriodic(period);
    }

    /**
     * Stop Odometery Method
     * 
     * <p> Stops tracking the robot position </p>
     */
    public void stopOdometery(){
        odometery.stop();
    }

    /**
     * Get Robot Position Method.
     * 
     * @return The position of the robot.
     */
    public RobotPos getRobotPos(){
        return new RobotPos(x, y, theta);
    }

    /**
     * Get Gyro Angle.
     * 
     * @return The fused heading from the sensors.
     */
    public double getGyroAngle(){
        return mGyro.getFusedHeading() - mGyroOffset;
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
                    new LogObject("XPos", x  * Constants.kFeet2Meters),
                    new LogObject("YPos", y  * Constants.kFeet2Meters),
                    new LogObject("Heading", theta)
                };
            }
        };
    }
}