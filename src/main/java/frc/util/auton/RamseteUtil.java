package frc.util.auton;

import frc.robot.Constants;
import frc.util.DriveSignal;
import frc.util.kinematics.pos.Pose2d;
import frc.util.kinematics.pos.Rotation2d;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * Ramsete Class.
 *
 * <p>A Utility that, when extended, creates a Ramsete Controller</p>
 * <p>It is important to note that this Controller is meant to be reused with multiple paths. I.E. Do not create
 * a new Controller for every Trajectory</p>
 *
 * The functionality of this Controller is based on this paper:
 * https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
 *
 * @author Avidh Bavkar [avidh@team4414.com
 * @author JJ Sessa [jonathan@team4414.co
 */
public abstract class RamseteUtil {

    private final double kTimestep;
    private static final double kZeta = 0.9; //damp (0.7)
    private static final double kB = 0.1; //aggresive (1.5)

    public enum Status{
        STANDBY,    //Robot is finished following a path and waiting for a new one.
        TRACKING    //Robot is currently busy tracking a path.
    }
    /**
     * Differentiable Robot Position Inner Class.
     *
     * <p>
     * Automatically stores three sequential {@link Pose2d} Objects and uses these to calculate the first
     * and second derivatives of the position
     * </p>
     */
    protected class DifferentiablePose2d {

        private Pose2d mCurrentPos;
        private Pose2d mPrevPos;
        private Pose2d mPrevPrevPos;

        public DifferentiablePose2d(Pose2d initPos){
            mPrevPos = new Pose2d();
            mPrevPrevPos = new Pose2d();
            update(initPos);
        }

        /**
         * Update Method.
         *
         * <p> Pass in the current position </p>
         */
        public void update(Pose2d currentPos){
            mPrevPrevPos = mPrevPos;
            mPrevPos = mCurrentPos;
            mCurrentPos = currentPos;
        }

        /**
         * Get Position Method.
         *
         * @return The current position.
         */
        public Pose2d getPos(){
            return mCurrentPos;
        }

        /**
         * Get Derivative Method.
         *
         * @return The first derivative of position.
         */
        public Pose2d getDeriv(){
            //slope of current and previous is approx. derivative.
            return new Pose2d(
                    getSlope(mCurrentPos.getTranslation().x(), mPrevPos.getTranslation().x()),
                    getSlope(mCurrentPos.getTranslation().y(), mPrevPos.getTranslation().y()),
                    Rotation2d.fromRadians(getSlope(mCurrentPos.getRotation().getRadians(), mPrevPos.getRotation().getRadians()))
            );
        }

        /**
         * Get Second Derivative Method.
         *
         * @return The second derivative of position.
         */
        public Pose2d getSecondDeriv(){
            //slope of the slope between three points is approx. second derivative.
            return new Pose2d(
                    getSlope(getSlope(mCurrentPos.getTranslation().x(), mPrevPos.getTranslation().x()),
                            getSlope(mPrevPos.getTranslation().x(), mPrevPrevPos.getTranslation().x())),
                    getSlope(getSlope(mCurrentPos.getTranslation().y(), mPrevPos.getTranslation().y()),
                            getSlope(mPrevPos.getTranslation().y(), mPrevPrevPos.getTranslation().y())),
                    
                    Rotation2d.fromRadians(
                        getSlope(getSlope(mCurrentPos.getRotation().getRadians(), mPrevPos.getRotation().getRadians()),
                                getSlope(mPrevPos.getRotation().getRadians(), mPrevPrevPos.getRotation().getRadians()))
                    )
            );
        }

        private double getSlope(double curr, double prev){
            return (curr - prev) / kTimestep;
        }


    }

    public Trajectory path;
    public int mSegCount;
    public static Status status = Status.STANDBY;

    protected DifferentiablePose2d mGoal;
    private Pose2d mPos;

    private double mConstant, mAngleError, ramv, ramw;

    private final double kWheelBase;

    public RamseteUtil(double wheelBase, double timeStep){
        mSegCount = -1; //-1 used as an invalid number
        mPos = getPose2d();
        kWheelBase = wheelBase;
        kTimestep = timeStep;
        mGoal = new DifferentiablePose2d(new Pose2d());
    }

    /**
     * Update Method.
     *
     * <p>Expected to be called once per specified timestep.</p>
     */
    public void update(){

        if (path == null || mSegCount >= path.length()){
            //if the path is null or you are done tracking one, reset the controller and do not continue.
            path = null;
            mConstant = 0;
            mAngleError = 0;
            ramv = 0;
            ramw = 0;
            status = Status.STANDBY;
            mSegCount = -1;
            return;
        }

        //otherwise you are tracking so update your values.
        status = Status.TRACKING;
        mGoal.update(new Pose2d(
                path.get(mSegCount).x * Constants.kFeet2Meters,
                path.get(mSegCount).y * Constants.kFeet2Meters,
                Rotation2d.fromRadians(path.get(mSegCount).heading)
        ));
        mPos = getPose2d();
        mPos = new Pose2d(
            mPos.getTranslation().x()  * Constants.kFeet2Meters,
            mPos.getTranslation().y()  * Constants.kFeet2Meters,
            mPos.getRotation()
        );
 

        mAngleError = Pathfinder.d2r(Pathfinder.boundHalfDegrees(Pathfinder.r2d(mGoal.getPos().getRotation().getRadians() - 
                      mPos.getRotation().getRadians())));

        //Constant Equation from the paper.
        mConstant = 2.0 * kZeta *
                Math.sqrt(Math.pow(mGoal.getDeriv().getRotation().getRadians(), 2.0) +
                kB * Math.pow((path.get(mSegCount).velocity * Constants.kFeet2Meters), 2.0));

        //Eq. 5.12!
        ramv =  path.get(mSegCount).velocity * Constants.kFeet2Meters * Math.cos(mAngleError) +
                mConstant * (Math.cos(mPos.getRotation().getRadians()) * 
                (mGoal.mCurrentPos.getTranslation().x() - mPos.getTranslation().x()) +
                Math.sin(mPos.getRotation().getRadians()) * (mGoal.mCurrentPos.getTranslation().y() - mPos.getTranslation().y()));

        //ramv = vd * Math.cos(eAngle) + k1 * (Math.cos(angle) * (gx - rx) + Math.sin(angle) * (gy - ry));

        ramw =  mGoal.getDeriv().getRotation().getRadians() + kB * path.get(mSegCount).velocity * Constants.kFeet2Meters *
                (Math.sin(mAngleError) / (mAngleError)) * (Math.cos(mPos.getRotation().getRadians()) *
                (mGoal.mCurrentPos.getTranslation().y() - mPos.getTranslation().y()) - Math.sin(mPos.getRotation().getRadians()) *
                (mGoal.mCurrentPos.getTranslation().x() - mPos.getTranslation().x())) + mConstant * (mAngleError);

        //ramw = wd + kb * vd * (Math.sin(eAngle) / (eAngle)) * (Math.cos(angle) * (gy - ry) - Math.sin(angle) * (gx - rx)) + k1 * (eAngle);


        mSegCount ++;
    }

    /**
     * Track Path Method.
     *
     * @param path The desired trajectory for the robot to follow.
     */
    public void trackPath(Trajectory path){
        this.path = path;
        mSegCount = 0;
    }

    /**
     * Update State Method.
     *
     * <p>Forces an update of state</p>
     */
    public void updateState(){
        status = (path == null || mSegCount >= path.length()) ? Status.STANDBY : Status.TRACKING;
    }

    /**
     * Get Velocities Method.
     *
     * @return A Velocity DriveSignal to apply to the drivetrain.
     */
    public DriveSignal getVels(){
        return new DriveSignal(
                Constants.kMeters2Feet* (ramv - ramw * (kWheelBase * Constants.kFeet2Meters) / 2),
                Constants.kMeters2Feet *(ramv + ramw * (kWheelBase * Constants.kFeet2Meters) / 2)
        );
    }

    /**
     * Get Status Method.
     *
     * @return The {@link Status} of the controller.
     */
    public Status getStatus(){
        return status;
    }

    /**
     * Get Robot Position Method.
     *
     * @return The current position of the robot.
     */
    public abstract Pose2d getPose2d();
}
