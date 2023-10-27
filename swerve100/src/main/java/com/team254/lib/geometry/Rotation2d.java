package com.team254.lib.geometry;

import com.team254.lib.util.Util;

public class Rotation2d extends edu.wpi.first.math.geometry.Rotation2d implements State<Rotation2d> {
    public Rotation2d() {
        super();
    }

    public Rotation2d(double radians) {
        super(radians);
    }

    public Rotation2d(double x, double y) {
        super(x, y);
    }

    public Rotation2d(final edu.wpi.first.math.geometry.Rotation2d other) {
        super(other.getRadians());
    }

    public static Rotation2d fromRadians(double angle_radians) {
        return new Rotation2d(angle_radians);
    }

    public static Rotation2d fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public Rotation2d unaryMinus() {
        return new Rotation2d(super.unaryMinus());
    }

    public Rotation2d times(double scalar) {
        return new Rotation2d(super.times(scalar));
    }

    public Rotation2d rotateBy(final Rotation2d other) {
        return new Rotation2d(super.rotateBy(other));
    }

    public Rotation2d normal() {
        return new Rotation2d(getRadians() - Math.PI / 2.0);
    }

    public Rotation2d flip() {
        return new Rotation2d(getRadians() + Math.PI);
    }

    public boolean isParallel(final Rotation2d other) {
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

    public Translation2d toTranslation() {
        return new Translation2d(getCos(), getSin());
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
    public Rotation2d interpolate2(final Rotation2d other,
            double x) {
        if (x <= 0.0) {
            return new Rotation2d(this);
        } else if (x >= 1.0) {
            return new Rotation2d(other);
        }
        double angle_diff = unaryMinus().rotateBy(other).getRadians();
        return this.rotateBy(Rotation2d.fromRadians(angle_diff * x));
    }

    @Override
    public double distance(final Rotation2d other) {
        return unaryMinus().rotateBy(other).getRadians();
    }

    @Override
    public Rotation2d add(Rotation2d other) {
        return this.rotateBy(other);
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Rotation2d)) {
            return false;
        }
        return distance((Rotation2d) other) < Util.kEpsilon;
    }

    public Rotation2d getRotation() {
        return this;
    }
}
