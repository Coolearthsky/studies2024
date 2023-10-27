package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dState extends Rotation2d implements State<Rotation2dState> {
    public Rotation2dState() {
        super();
    }

    public Rotation2dState(double radians) {
        super(radians);
    }

    public Rotation2dState(double x, double y) {
        super(x, y);
    }

    public Rotation2dState(final Rotation2d other) {
        super(other.getRadians());
    }

    public static Rotation2dState fromRadians(double angle_radians) {
        return new Rotation2dState(angle_radians);
    }

    public static Rotation2dState fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public Rotation2dState unaryMinus() {
        return new Rotation2dState(super.unaryMinus());
    }

    public Rotation2dState times(double scalar) {
        return new Rotation2dState(super.times(scalar));
    }

    public Rotation2dState rotateBy(final Rotation2dState other) {
        return new Rotation2dState(super.rotateBy(other));
    }

    public Rotation2dState normal() {
        return new Rotation2dState(getRadians() - Math.PI / 2.0);
    }

    public Rotation2dState flip() {
        return new Rotation2dState(getRadians() + Math.PI);
    }

    public boolean isParallel(final Rotation2dState other) {
        if (hasRadians() && other.hasRadians()) {
            return Util.epsilonEquals(getRadians(), other.getRadians())
                    || Util.epsilonEquals(getRadians(), WrapRadians(other.getRadians() + Math.PI));
        } else if (hasTrig() && other.hasTrig()) {
            return Util.epsilonEquals(getSin(), other.getSin()) && Util.epsilonEquals(getCos(), other.getCos());
        } else {
            // Use public, checked version.
            return Util.epsilonEquals(getRadians(), other.getRadians())
                    || Util.epsilonEquals(getRadians(), WrapRadians(other.getRadians() + Math.PI));
        }
    }

    public Translation2dState toTranslation() {
        return new Translation2dState(getCos(), getSin());
    }

    protected double WrapRadians(double radians) {
        final double k2Pi = 2.0 * Math.PI;
        radians = radians % k2Pi;
        radians = (radians + k2Pi) % k2Pi;
        if (radians > Math.PI)
            radians -= k2Pi;
        return radians;
    }

    private synchronized boolean hasTrig() {
        return !Double.isNaN(getSin()) && !Double.isNaN(getCos());
    }

    private synchronized boolean hasRadians() {
        return !Double.isNaN(getRadians());
    }

    @Override
    public Rotation2dState interpolate2(final Rotation2dState other,
            double x) {
        if (x <= 0.0) {
            return new Rotation2dState(this);
        } else if (x >= 1.0) {
            return new Rotation2dState(other);
        }
        double angle_diff = unaryMinus().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2dState.fromRadians(angle_diff * x));
    }

    @Override
    public double distance(final Rotation2dState other) {
        return unaryMinus().rotateBy(other).getRadians();
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
