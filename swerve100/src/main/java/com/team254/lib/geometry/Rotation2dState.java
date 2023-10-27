package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dState implements State<Rotation2dState> {
    final Rotation2d rotation2d;

    public Rotation2d get() {
        return rotation2d;
    }

    public Rotation2dState() {
        this(new Rotation2d());
    }

    public Rotation2dState(double radians) {
        this(new Rotation2d(radians));
    }

    public Rotation2dState(double x, double y) {
        this(new Rotation2d(x,y));
    }

    public Rotation2dState(final Rotation2d other) {
        this.rotation2d = other;
    }

    public static Rotation2dState fromRadians(double angle_radians) {
        return new Rotation2dState(angle_radians);
    }

    public static Rotation2dState fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public Rotation2dState unaryMinus() {
        return new Rotation2dState(rotation2d.unaryMinus());
    }

    public Rotation2dState times(double scalar) {
        return new Rotation2dState(rotation2d.times(scalar));
    }

    public Rotation2dState rotateBy(final Rotation2dState other) {
        return new Rotation2dState(rotation2d.rotateBy(other.rotation2d));
    }

    public Rotation2dState rotateBy(final Rotation2d other) {
        return new Rotation2dState(rotation2d.rotateBy(other));
    }

    public Rotation2dState normal() {
        return new Rotation2dState(rotation2d.getRadians() - Math.PI / 2.0);
    }

    public Rotation2dState flip() {
        return new Rotation2dState(rotation2d.getRadians() + Math.PI);
    }

    public boolean isParallel(final Rotation2dState other) {
        if (hasRadians() && other.hasRadians()) {
            return Util.epsilonEquals(rotation2d.getRadians(), other.rotation2d.getRadians())
                    || Util.epsilonEquals(rotation2d.getRadians(), WrapRadians(other.rotation2d.getRadians() + Math.PI));
        } else if (hasTrig() && other.hasTrig()) {
            return Util.epsilonEquals(rotation2d.getSin(), other.rotation2d.getSin()) && Util.epsilonEquals(rotation2d.getCos(), other.rotation2d.getCos());
        } else {
            // Use public, checked version.
            return Util.epsilonEquals(rotation2d.getRadians(), other.rotation2d.getRadians())
                    || Util.epsilonEquals(rotation2d.getRadians(), WrapRadians(other.rotation2d.getRadians() + Math.PI));
        }
    }

    public static double WrapRadians(double radians) {
        final double k2Pi = 2.0 * Math.PI;
        radians = radians % k2Pi;
        radians = (radians + k2Pi) % k2Pi;
        if (radians > Math.PI)
            radians -= k2Pi;
        return radians;
    }

    private synchronized boolean hasTrig() {
        return !Double.isNaN(rotation2d.getSin()) && !Double.isNaN(rotation2d.getCos());
    }

    private synchronized boolean hasRadians() {
        return !Double.isNaN(rotation2d.getRadians());
    }

    @Override
    public Rotation2dState interpolate2(final Rotation2dState other,
            double x) {
        if (x <= 0.0) {
            return new Rotation2dState(this.rotation2d);
        } else if (x >= 1.0) {
            return new Rotation2dState(other.rotation2d);
        }
        double angle_diff = unaryMinus().rotateBy(other).rotation2d.getRadians();
        return this.rotateBy(Rotation2dState.fromRadians(angle_diff * x));
    }

    @Override
    public double distance(final Rotation2dState other) {
        return unaryMinus().rotateBy(other).rotation2d.getRadians();
    }

    @Override
    public Rotation2dState add(Rotation2dState other) {
        return this.rotateBy(other);
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Rotation2dState)) {
            return false;
        }
        return distance((Rotation2dState) other) < Util.kEpsilon;
    }

    public Rotation2dState getRotation() {
        return this;
    }
}
