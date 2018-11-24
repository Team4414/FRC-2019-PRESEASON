package frc.util.auton;

import frc.robot.Constants;
import frc.util.DriveSignal;
import frc.util.kinematics.RobotPos;
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
 * @author Avidh Bavkar [avidh@team4414.com]
 * @author JJ Sessa [jonathan@team4414.com]
 */
public abstract class Ramsete {

    private final double kTimestep;
    private static final double kZeta = 0.01;
    private static final double kB = 0.7;

    public enum Status{
        STANDBY,    //Robot is finished following a path and waiting for a new one.
        TRACKING    //Robot is currently busy tracking a path.
    }

    /**
     * Differentiable Robot Position Inner Class.
     *
     * <p>
     * Automatically stores three sequential {@link RobotPos} Objects and uses these to calculate the first
     * and second derivatives of the position
     * </p>
     */
    protected class DifferentiableRobotPos {

        private RobotPos mCurrentPos;
        private RobotPos mPrevPos;
        private RobotPos mPrevPrevPos;

        public DifferentiableRobotPos(RobotPos initPos){
            mPrevPos = new RobotPos(0,0,0);
            mPrevPrevPos = new RobotPos(0,0,0);
            update(initPos);
        }

        /**
         * Update Method.
         *
         * <p> Pass in the current position </p>
         */
        public void update(RobotPos currentPos){
            mPrevPrevPos = mPrevPos;
            mPrevPos = mCurrentPos;
            mCurrentPos = currentPos;
        }

        /**
         * Get Position Method.
         *
         * @return The current position.
         */
        public RobotPos getPos(){
            return mCurrentPos;
        }

        /**
         * Get Derivative Method.
         *
         * @return The first derivative of position.
         */
        public RobotPos getDeriv(){
            //slope of current and previous is approx. derivative.
            return new RobotPos(
                    getSlope(mCurrentPos.getX(), mPrevPos.getX()),
                    getSlope(mCurrentPos.getY(), mPrevPos.getY()),
                    getSlope(mCurrentPos.getHeading(), mPrevPos.getHeading())
            );
        }

        /**
         * Get Second Derivative Method.
         *
         * @return The second derivative of position.
         */
        public RobotPos getSecondDeriv(){
            //slope of the slope between three points is approx. second derivative.
            return new RobotPos(
                    getSlope(getSlope(mCurrentPos.getX(), mPrevPos.getX()),
                            getSlope(mPrevPos.getX(), mPrevPrevPos.getX())),
                    getSlope(getSlope(mCurrentPos.getY(), mPrevPos.getY()),
                            getSlope(mPrevPos.getY(), mPrevPrevPos.getY())),
                    getSlope(getSlope(mCurrentPos.getHeading(), mPrevPos.getHeading()),
                            getSlope(mPrevPos.getHeading(), mPrevPrevPos.getHeading()))
            );
        }

        private double getSlope(double curr, double prev){
            return (curr - prev) / kTimestep;
        }


    }

    private Trajectory path;
    private int mSegCount;
    public static Status status = Status.STANDBY;

    protected DifferentiableRobotPos mGoal;
    private RobotPos mPos;

    private double mConstant, mAngleError, ramv, ramw;

    private final double kWheelBase;

    public Ramsete(double wheelBase, double timeStep){
        mSegCount = -1; //-1 used as an invalid number
        mPos = getRobotPos();
        kWheelBase = wheelBase;
        kTimestep = timeStep;
        mGoal = new DifferentiableRobotPos(new RobotPos(0,0,0));
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
        mGoal.update(new RobotPos(
                path.get(mSegCount).x * Constants.kFeet2Meters,
                path.get(mSegCount).y * Constants.kFeet2Meters,
                path.get(mSegCount).heading
        ));
        mPos = getRobotPos();
        mPos = new RobotPos(
            mPos.getX(),
            mPos.getY(),
            Pathfinder.d2r(mPos.getHeading())
        );

        mAngleError = Pathfinder.d2r(Pathfinder.boundHalfDegrees(mGoal.getPos().getHeading() - mPos.getHeading()));

        //Constant Equation from the paper.
        mConstant = 2.0 * kZeta *
                Math.sqrt(Math.pow(mGoal.getDeriv().getHeading(), 2.0) +
                kB * Math.pow(path.get(mSegCount).velocity, 2.0));

        //Eq. 5.12!
        ramv =  path.get(mSegCount).velocity * Math.cos(mAngleError) +
                mConstant * (Math.cos(mPos.getHeading()) * (mGoal.mCurrentPos.getX() - mPos.getX()) +
                Math.sin(mPos.getHeading()) * (mGoal.mCurrentPos.getY() - mPos.getY()));

        ramw =  mGoal.getDeriv().getHeading() + kB * path.get(mSegCount).velocity *
                (Math.sin(mAngleError) / (mAngleError)) * (Math.cos(mPos.getHeading()) *
                (mGoal.mCurrentPos.getY() - mPos.getY()) - Math.sin(mPos.getHeading()) *
                (mGoal.mCurrentPos.getX() - mPos.getX())) + mConstant * (mAngleError);

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
                (ramv - (ramw * (kWheelBase / 2))),
                (ramv + (ramw * (kWheelBase / 2)))
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
    public abstract RobotPos getRobotPos();
}
