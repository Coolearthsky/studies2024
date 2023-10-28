package com.team254.lib.geometry;

import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pose2dState implements State<Pose2dState> {
    public final Pose2d pose2d;

    public Pose2d get() {
        return pose2d;
    }

    public Pose2dState() {
        pose2d = new Pose2d();
    }

    public Pose2dState(double x, double y, final Rotation2dState rotation) {
        pose2d = new Pose2d(x, y, rotation.get());
    }

    public Pose2dState(double x, double y, final Rotation2d rotation) {
        pose2d = new Pose2d(x, y, rotation);
    }

    public Pose2dState(final Translation2dState translation, final Rotation2dState rotation) {
        this(translation.get(), rotation.get());
    }

    public Pose2dState(final Translation2d translation, final Rotation2dState rotation) {
        this(translation, rotation.get());
    }

    public Pose2dState(final Translation2d translation, final Rotation2d rotation) {
        pose2d = new Pose2d(translation, rotation);
    }

    public Pose2dState(final Pose2dState other) {
        this(other.pose2d.getTranslation(), other.pose2d.getRotation());
    }

    public Pose2dState(final Pose2d other) {
        this(other.getTranslation(), other.getRotation());
    }

    public Translation2dState getTranslation2dState() {
        return new Translation2dState(pose2d.getTranslation());
    }

    @Override
    public Pose2dState add(Pose2dState other) {
        return new Pose2dState(this.transformBy(other.pose2d));
    }

    public Pose2d transformBy(final Pose2d other) {
        return get().transformBy(new Transform2d(other.getTranslation(), other.getRotation()));
    }

    public Pose2dState inverse() {
        Rotation2d rotation_inverted = pose2d.getRotation().unaryMinus();
        return new Pose2dState(pose2d.getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);
    }

    public boolean isColinear(final Pose2dState other) {
        if (!new Rotation2dState(pose2d.getRotation()).isParallel(new Rotation2dState(other.pose2d.getRotation())))
            return false;
        final Twist2dWrapper twist = GeometryUtil.slog(
            new Pose2dState(inverse().transformBy(other.pose2d)));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public boolean epsilonEquals(final Pose2dState other, double epsilon) {
        return getTranslation2dState().epsilonEquals(other.getTranslation2dState(), epsilon)
                && new Rotation2dState(pose2d.getRotation()).isParallel(new Rotation2dState(other.pose2d.getRotation()));
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2dState interpolate2(final Pose2dState other, double x) {
        return new Pose2dState(pose2d.interpolate(other.pose2d, x));
    }

    @Override
    public double distance(final Pose2dState other) {
        return GeometryUtil.slog(new Pose2dState(inverse().transformBy(other.pose2d))).norm();
    }

    public Pose2dState getPose() {
        return this;
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof Pose2dState)) {
            return false;
        }
        return pose2d.equals(((Pose2dState)other).pose2d);
    }
}
