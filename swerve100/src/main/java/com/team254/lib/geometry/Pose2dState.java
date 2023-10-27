package com.team254.lib.geometry;

import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pose2dState extends Pose2d implements State<Pose2dState> {
    public Pose2dState() {
    }

    public Pose2dState(double x, double y, final Rotation2dState rotation) {
        super(x, y, rotation);
    }

    public Pose2dState(final Translation2dState translation, final Rotation2dState rotation) {
        super(translation.get(), rotation);
    }

    public Pose2dState(final Translation2d translation, final Rotation2dState rotation) {
        super(translation, rotation);
    }

    public Pose2dState(final Translation2d translation, final Rotation2d rotation) {
        super(translation, rotation);
    }

    public Pose2dState(final Pose2dState other) {
        super(new Translation2dState(other.getTranslation()).get(), new Rotation2dState(other.getRotation()));
    }

    public Pose2dState(final Pose2d other) {
        super(new Translation2dState(other.getTranslation()).get(), new Rotation2dState(other.getRotation()));

    }

    public static Pose2dState fromTranslation(final Translation2dState translation) {
        return new Pose2dState(translation, new Rotation2dState());
    }

    public static Pose2dState fromRotation(final Rotation2dState rotation) {
        return new Pose2dState(new Translation2dState(), rotation);
    }

    public static Pose2dState sexp(final Twist2dWrapper delta) {
        return new Pose2dState(new Pose2dState().exp(delta));
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist2dWrapper slog(final Pose2dState transform) {
        Pose2dState base = new Pose2dState();
        return new Twist2dWrapper(base.log(transform));
    }

    public Translation2dState getTranslation2dState() {
        return new Translation2dState(super.getTranslation());
    }

    @Override
    public Rotation2dState getRotation() {
        return new Rotation2dState(super.getRotation());
    }

    public Pose2dState rotateBy(Rotation2dState other) {
        return this.transformBy(new Pose2dState(GeometryUtil.kTranslation2dIdentity, other));
    }

    @Override
    public Pose2dState add(Pose2dState other) {
        return this.transformBy(other);
    }

    public Pose2dState transformBy(final Pose2dState other) {
        return new Pose2dState(getTranslation().plus(other.getTranslation().rotateBy(getRotation())),
                getRotation().rotateBy(other.getRotation()));
    }

    public Pose2dState transformBy(Transform2d other) {
        return new Pose2dState(
                getTranslation().plus(other.getTranslation().rotateBy(getRotation())),
                getRotation().rotateBy(other.getRotation()));
    }

    public Pose2dState inverse() {
        Rotation2dState rotation_inverted = getRotation().unaryMinus();
        return new Pose2dState(getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);
    }

    public boolean isColinear(final Pose2dState other) {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist2dWrapper twist = slog(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public boolean epsilonEquals(final Pose2dState other, double epsilon) {
        return getTranslation2dState().epsilonEquals(other.getTranslation2dState(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2dState interpolate2(final Pose2dState other, double x) {
        return new Pose2dState(super.interpolate(other, x));
    }

    @Override
    public double distance(final Pose2dState other) {
        return Pose2dState.slog(inverse().transformBy(other)).norm();
    }

    public Pose2dState getPose() {
        return this;
    }

    public Pose2dState mirror() {
        return new Pose2dState(new Translation2dState(getTranslation().getX(), -getTranslation().getY()),
                getRotation().unaryMinus());
    }
}
