package com.team254.lib.geometry;

import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Transform2d;

public class Pose2d extends edu.wpi.first.math.geometry.Pose2d implements State<Pose2d> {
    public Pose2d() {
    }

    public Pose2d(double x, double y, final Rotation2d rotation) {
        super(x, y, rotation);
    }

    public Pose2d(final Translation2d translation, final Rotation2d rotation) {
        super(translation, rotation);
    }

    public Pose2d(final edu.wpi.first.math.geometry.Translation2d translation,
            final edu.wpi.first.math.geometry.Rotation2d rotation) {
        super(translation, rotation);
    }

    public Pose2d(final Pose2d other) {
        super(new Translation2d(other.getTranslation()), new Rotation2d(other.getRotation()));
    }

    public Pose2d(final edu.wpi.first.math.geometry.Pose2d other) {
        super(new Translation2d(other.getTranslation()), new Rotation2d(other.getRotation()));

    }

    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Pose2d fromRotation(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    public static Pose2d sexp(final Twist2d delta) {
        return new Pose2d(new Pose2d().exp(delta));
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist2d slog(final Pose2d transform) {
        Pose2d base = new Pose2d();
        return new Twist2d(base.log(transform));
    }

    public Translation2d getTranslation() {
        return new Translation2d(super.getTranslation());
    }

    @Override
    public Rotation2d getRotation() {
        return new Rotation2d(super.getRotation());
    }

    public Pose2d rotateBy(Rotation2d other) {
        return this.transformBy(new Pose2d(GeometryUtil.kTranslation2dIdentity, other));
    }

    @Override
    public Pose2d add(Pose2d other) {
        return this.transformBy(other);
    }

    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(getTranslation().plus(other.getTranslation().rotateBy(getRotation())),
                getRotation().rotateBy(other.getRotation()));
    }

    public Pose2d transformBy(Transform2d other) {
        return new Pose2d(
                getTranslation().plus(other.getTranslation().rotateBy(getRotation())),
                getRotation().rotateBy(other.getRotation()));
    }

    public Pose2d inverse() {
        edu.wpi.first.math.geometry.Rotation2d rotation_inverted = getRotation().unaryMinus();
        return new Pose2d(getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);
    }

    public boolean isColinear(final Pose2d other) {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist2d twist = slog(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public boolean epsilonEquals(final Pose2d other, double epsilon) {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2d interpolate2(final Pose2d other, double x) {
        return new Pose2d(super.interpolate(other, x));
    }

    @Override
    public double distance(final Pose2d other) {
        return Pose2d.slog(inverse().transformBy(other)).norm();
    }

    public Pose2d getPose() {
        return this;
    }

    public Pose2d mirror() {
        return new Pose2d(new Translation2d(getTranslation().getX(), -getTranslation().getY()),
                getRotation().unaryMinus());
    }
}
