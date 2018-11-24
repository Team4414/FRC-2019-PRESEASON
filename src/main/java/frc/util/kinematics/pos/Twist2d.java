package frc.util.kinematics.pos;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to turn this into a translation and rotation.
 * <p>
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
public class Twist2d {
    protected static final Twist2d kIdentity = new Twist2d(0.0, 0.0);

    public static final Twist2d identity() {
        return kIdentity;
    }

    public final double dpos;
    public final double theta; // Radians!

    public Twist2d(double dpos, double theta) {
        this.dpos = dpos;
        this.theta = theta;
    }

    public static Twist2d fromWheels(double dleft, double dright, double theta) {
        return new Twist2d((dleft+dright)/2,theta);
    }

    public Twist2d scaled(double scale) {
        return new Twist2d(dpos * scale, theta * scale);
    }

}