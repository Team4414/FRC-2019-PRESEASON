package frc.util.kinematics.pos;

/**
 * Represents a 2d pose (rigid transform) containing translational and rotational elements.
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class Pose2d {
    protected static final Pose2d kIdentity = new Pose2d();

    public static final Pose2d identity() {
        return kIdentity;
    }


    protected final Translation2d translation_;
    protected final Rotation2d rotation_;

    public Pose2d() {
        translation_ = new Translation2d();
        rotation_ = new Rotation2d();
    }

    public Pose2d(double x, double y, final Rotation2d rotation) {
        translation_ = new Translation2d(x, y);
        rotation_ = rotation;
    }

    public Pose2d(final Translation2d translation, final Rotation2d rotation) {
        translation_ = translation;
        rotation_ = rotation;
    }

    public Pose2d(final Pose2d other) {
        translation_ = new Translation2d(other.translation_);
        rotation_ = new Rotation2d(other.rotation_);
    }

    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Pose2d fromRotation(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    public Translation2d getTranslation() {
        return translation_;
    }


    public Rotation2d getRotation() {
        return rotation_;
    }

    /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                rotation_.rotateBy(other.rotation_));
    }


    /**
     * Uses differential calculus to estimate a constanct curvature position
     * 
     * @param other the twist.
     * @return This transtlation
     */
    public Pose2d transformBy(final Twist2d other) {
        double dx = other.dpos * Rotation2d.fromRadians(other.theta).cos();
        double dy = other.dpos * Rotation2d.fromRadians(other.theta).sin();
        return new Pose2d(translation_.translateBy(new Translation2d(dx,dy)),
                Rotation2d.fromRadians(other.theta));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public Pose2d inverse() {
        Rotation2d rotation_inverted = rotation_.inverse();
        return new Pose2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    public Pose2d normal() {
        return new Pose2d(translation_, rotation_.normal());
    }

    public Pose2d mirror() {
        return new Pose2d(new Translation2d(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
    }
}