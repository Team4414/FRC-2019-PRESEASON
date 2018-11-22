package frc.util.kinematics;
/**
 * Forward Kinematics Class.
 *
 * <P>Utility to handle the mathematics of Forward Kinematics</P>
 * 
 * @author Avidh Bavkar <avidh@team4414.com>
 */
public abstract class OdometeryUtil{
    /**
    * Radius of the drive wheels
    */
   private final double kRadius;
    /**
    * Distance between the centers of both wheels
    */
   private final double kWheelBase;
    /**
    * The amount, in seconds, between calls of the "update" method
    */
   private final double kTimeStep;
    /**
    * Robot X Position
    */
   private double mXpos;
    /**
    * Robot Y Position
    */
   private double mYpos;
    /**
    * Robot Heading
    */
   private double mHeading;
    /**
    * Constructor.
    *
    * @param wheelRadius Radius of the drive wheels.
    * @param wheelBaseLength Distance between the centers of both wheels.
    */
   public OdometeryUtil(double wheelRadius, double wheelBaseLength, double timestep){
       kRadius = wheelRadius;
       kWheelBase = wheelBaseLength;
       kTimeStep = timestep;
   }
    /**
    * Constructor.
    *
    * @param wheelRadius Radius of the drive wheels.
    * @param wheelBaseLength Distance between the centers of both wheels.
    * @param xPos Initial X Position of the Robot.
    * @param yPos Initial Y Position of the Robot.
    * @param heading Initial Heading of the Robot.
    */
   public OdometeryUtil(double wheelRadius, double wheelBaseLength, double timestep, double xPos, double yPos, double heading){
       this(wheelRadius, wheelBaseLength, timestep);
       mXpos = xPos;
       mYpos = yPos;
       mHeading = heading;
   }
    /**
    * Update Method.
    *
    * <P>Integrates coordinates and heading of the robot. Expected to be called once per specified TIMESTEP</P>
    */
   public void update(){
       mXpos += kTimeStep * getDeltaX();
       System.out.println(getDeltaX() + " | " + getDeltaY() + " | " + getInputHeading());
       mYpos += kTimeStep * getDeltaY();
       mHeading = getInputHeading();
   }
    /**
    * Get X Position Method.
    * @return The current X position of the robot.
    */
   public double getXPosition(){ return mXpos; }
    /**
    * Get Y Position Method.
    * @return The current Y position of the robot.
    */
   public double getYPosition(){ return mYpos; }
    /**
    * Get Delta X Method.
    *
    * Calculates the change in the X Position of the Robot with respect to wheel velocities and heading.
    *
    * @return The change in X Position.
    */
   private double getDeltaX(){
       //Check this link for the equations:
       return ((getLeftWheelVelocity() + getRightWheelVelocity())/2) * Math.cos((mHeading/360) * (2*Math.PI));
   }
    /**
    * Get Delta Y Method.
    *
    * Calculates the change in the Y Position of the Robot with respect to wheel velocities and heading.
    *
    * @return The change in Y Position.
    */
   private double getDeltaY(){
       return ((getLeftWheelVelocity() + getRightWheelVelocity())/2) * Math.sin((mHeading/360) * (2*Math.PI));
   }
    /**
    * Get Left Wheel Velocity Method.
    *
    * @return The current measured velocity of the left wheel.
    */
   protected abstract double getLeftWheelVelocity();
    /**
    * Get Left Wheel Velocity Method.
    *
    * @return The current measured velocity of the right wheel.
    */
   protected abstract double getRightWheelVelocity();

   /**
    * Get Heading Method.

    * @return The measured Heading of the robot.
    */
   protected abstract double getInputHeading();
}