package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import java.text.DecimalFormat;

public class Twist2d extends edu.wpi.first.math.geometry.Twist2d {
    public Twist2d(double dx, double dy, double dtheta) {
        super(dx, dy, dtheta);
    }

    public Twist2d(edu.wpi.first.math.geometry.Twist2d other) {
        super(other.dx, other.dy, other.dtheta);
    }

    public Twist2d scaled(double scale) {
        return new Twist2d(dx * scale, dy * scale, dtheta * scale);
    }

    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    public double curvature() {
        if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
            return 0.0;
        return dtheta / norm();
    }

    public boolean epsilonEquals(final Twist2d other, double epsilon) {
        return Util.epsilonEquals(dx, other.dx, epsilon) &&
                Util.epsilonEquals(dy, other.dy, epsilon) &&
                Util.epsilonEquals(dtheta, other.dtheta, epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    }
}