package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Translation2d;

public class Translation2dState implements State<Translation2dState> {
    private final Translation2d translation2d;

    public Translation2dState(Translation2d translation2d) {
        this.translation2d = translation2d;
    }

    public Translation2dState() {
        this(new Translation2d());
    }

    public Translation2dState(double x, double y) {
        this(new Translation2d(x, y));
    }

    public Translation2dState(final Translation2dState other) {
        this(other.translation2d.getX(), other.translation2d.getY());
    }

    public Translation2dState(final Translation2dState start, final Translation2dState end) {
        this(end.translation2d.getX() - start.translation2d.getX(), end.translation2d.getY() - start.translation2d.getY());
    }

    public Translation2dState(final Translation2d start, final Translation2d end) {
        this(end.getX() - start.getX(), end.getY() - start.getY());
    }

    public Translation2d get() {
        return translation2d;
    }

    public Translation2dState plus(Translation2dState other) {
        return new Translation2dState(translation2d.plus(other.translation2d));
    }

    public Translation2dState unaryMinus() {
        return new Translation2dState(translation2d.unaryMinus());
    }

    public Translation2dState rotateBy(final Rotation2dState rotation) {
        return new Translation2dState(translation2d.rotateBy(rotation));
    }

    public Rotation2dState direction() {
        return new Rotation2dState(translation2d.getX(), translation2d.getY());
    }

    @Override
    public Translation2dState interpolate2(final Translation2dState other, double x) {
        if (x <= 0) {
            return new Translation2dState(this);
        } else if (x >= 1) {
            return new Translation2dState(other);
        }
        return extrapolate(other, x);
    }

    public Translation2dState extrapolate(final Translation2dState other, double x) {
        return new Translation2dState(x * (other.translation2d.getX() - translation2d.getX()) + translation2d.getX(), x * (other.translation2d.getY() - translation2d.getY()) + translation2d.getY());
    }

    public Translation2dState scale(double s) {
        return new Translation2dState(translation2d.getX() * s, translation2d.getY() * s);
    }

    public boolean epsilonEquals(final Translation2dState other, double epsilon) {
        return Util.epsilonEquals(translation2d.getX(), other.translation2d.getX(), epsilon) && Util.epsilonEquals(translation2d.getY(), other.translation2d.getY(), epsilon);
    }

    @Override
    public double distance(final Translation2dState other) {
        return translation2d.getDistance(other.translation2d);
    }

    @Override
    public Translation2dState add(Translation2dState other) {
        return new Translation2dState(plus(other));
    }

    public Translation2dState getTranslation() {
        return this;
    }
}
