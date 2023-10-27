package com.team254.lib.geometry;

import com.team254.lib.util.Util;

public class Translation2d extends edu.wpi.first.math.geometry.Translation2d implements State<Translation2d> {
    public Translation2d() {
        super();
    }

    public Translation2d(double x, double y) {
        super(x,y);
    }

    public Translation2d(final Translation2d other) {
        super(other.getX(), other.getY());
    }

    public Translation2d(final Translation2d start, final Translation2d end) {
        super(end.getX()-start.getX(),end.getY()-start.getY());
    }

    public Translation2d(final edu.wpi.first.math.geometry.Translation2d start, final edu.wpi.first.math.geometry.Translation2d end) {
        super(end.getX()-start.getX(),end.getY()-start.getY());
    }

    public Translation2d(final edu.wpi.first.math.geometry.Translation2d other) {
        super(other.getX(), other.getY());
    }

    public Translation2d plus(Translation2d other) {
        return new Translation2d(super.plus(other));
    }

    @Override
    public Translation2d unaryMinus() {
        return new Translation2d(super.unaryMinus());
    }

    public Translation2d rotateBy(final Rotation2d rotation) {
        return new Translation2d(super.rotateBy(rotation));
    }

    public Rotation2d direction() {
        return new Rotation2d(getX(), getY());
    }

    @Override
    public Translation2d interpolate2(final Translation2d other, double x) {
        if (x <= 0) {
            return new Translation2d(this);
        } else if (x >= 1) {
            return new Translation2d(other);
        }
        return extrapolate(other, x);
    }

    public Translation2d extrapolate(final Translation2d other, double x) {
        return new Translation2d(x * (other.getX() - getX()) + getX(), x * (other.getY() - getY()) + getY());
    }

    public Translation2d scale(double s) {
        return new Translation2d(getX() * s, getY() * s);
    }

    public boolean epsilonEquals(final Translation2d other, double epsilon) {
        return Util.epsilonEquals(getX(), other.getX(), epsilon) && Util.epsilonEquals(getY(), other.getY(), epsilon);
    }

    @Override
    public double distance(final Translation2d other) {
        return getDistance(other);
    }

    @Override
    public Translation2d add(Translation2d other) {
        return new Translation2d(plus(other));
    }

    public Translation2d getTranslation() {
        return this;
    }
}
