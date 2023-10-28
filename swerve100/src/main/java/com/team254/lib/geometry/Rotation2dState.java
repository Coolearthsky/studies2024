package com.team254.lib.geometry;

import org.team100.lib.geometry.GeometryUtil;

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


    public Rotation2d rotateBy(final Rotation2dState other) {
        return get().rotateBy(other.rotation2d);
    }

    public Rotation2dState rotateBy(final Rotation2d other) {
        return new Rotation2dState(get().rotateBy(other));
    }

    public Rotation2dState normal() {
        return new Rotation2dState(get().getRadians() - Math.PI / 2.0);
    }

    public Rotation2dState flip() {
        return new Rotation2dState(get().getRadians() + Math.PI);
    }

    @Override
    public Rotation2dState interpolate2(final Rotation2dState other,
            double x) {
        if (x <= 0.0) {
            return new Rotation2dState(this.get());
        } else if (x >= 1.0) {
            return new Rotation2dState(other.get());
        }
        double angle_diff = get().unaryMinus().rotateBy(other.get()).getRadians();
        return new Rotation2dState(this.rotateBy(GeometryUtil.fromRadians(angle_diff * x)));
    }

    @Override
    public double distance(final Rotation2dState other) {
        return get().unaryMinus().rotateBy(other.get()).getRadians();
    }

    // @Override
    // public Rotation2dState add(Rotation2dState other) {
    //     return this.rotateBy(other);
    // }

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
