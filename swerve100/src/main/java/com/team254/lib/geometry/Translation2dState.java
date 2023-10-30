package com.team254.lib.geometry;

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

    public Translation2d get() {
        return translation2d;
    }

    @Override
    public Translation2dState interpolate2(final Translation2dState other, double x) {
        // this is not used
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

    @Override
    public double distance(final Translation2dState other) {
        // this is not used
        return translation2d.getDistance(other.translation2d);
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof Translation2dState)) {
            return false;
        }
        return translation2d.equals(((Translation2dState)other).translation2d);
    }
}
