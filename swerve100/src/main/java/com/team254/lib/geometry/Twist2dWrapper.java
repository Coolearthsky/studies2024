package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Twist2d;

import java.text.DecimalFormat;

import org.team100.lib.geometry.GeometryUtil;

public class Twist2dWrapper extends Twist2d {
    public Twist2dWrapper(double dx, double dy, double dtheta) {
        super(dx, dy, dtheta);
    }

    public Twist2dWrapper(edu.wpi.first.math.geometry.Twist2d other) {
        super(other.dx, other.dy, other.dtheta);
    }

    public Twist2dWrapper scaled(double scale) {
        return new Twist2dWrapper(dx * scale, dy * scale, dtheta * scale);
    }

    public double curvature() {
        if (Math.abs(dtheta) < Util.kEpsilon && GeometryUtil.norm(this) < Util.kEpsilon)
            return 0.0;
        return dtheta / GeometryUtil.norm(this);
    }

    public boolean epsilonEquals(final Twist2dWrapper other, double epsilon) {
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