package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
 */
public class Translation2d extends edu.wpi.first.math.geometry.Translation2d implements State<Translation2d> {
    protected static final Translation2d kIdentity = new Translation2d();

    public static Translation2d identity() {
        return kIdentity;
    }

    // protected final double x_;
    // protected final double y_;

    public Translation2d() {
        super();
        // x_ = 0;
        // y_ = 0;
    }

    public Translation2d(double x, double y) {
        super(x,y);
        // x_ = x;
        // y_ = y;
    }

    public Translation2d(final Translation2d other) {
        super(other.getX(), other.getY());
        // x_ = other.x_;
        // y_ = other.y_;
    }

    public Translation2d(final Translation2d start, final Translation2d end) {
        super(end.getX()-start.getX(),end.getY()-start.getY());
        // x_ = end.x_ - start.x_;
        // y_ = end.y_ - start.y_;
    }

    public Translation2d(final edu.wpi.first.math.geometry.Translation2d start, final edu.wpi.first.math.geometry.Translation2d end) {
        super(end.getX()-start.getX(),end.getY()-start.getY());
        // x_ = end.x_ - start.x_;
        // y_ = end.y_ - start.y_;
    }

    public Translation2d(final edu.wpi.first.math.geometry.Translation2d other) {
        super(other.getX(), other.getY());
        // x_= other.getX();
        // y_ = other.getY();
    }


    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    // public double norm() {
    //     return Math.hypot(x_, y_);
    // }

    // public double norm2() {
    //     return x_ * x_ + y_ * y_;
    // }

    // public double x() {
    //     return x_;
    // }

    // public double y() {
    //     return y_;
    // }

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    // public Translation2d translateBy(final Translation2d other) {
    //     return new Translation2d(x_ + other.x_, y_ + other.y_);
    // }

    // public Translation2d translateBy(final edu.wpi.first.math.geometry.Translation2d other) {
    //     return new Translation2d(x_ + other.getX(), y_ + other.getY());
    // }
    

    public Translation2d plus(Translation2d other) {
        return new Translation2d(super.plus(other));
        // return new Translation2d(x_ + other.x(), y_ + other.y());
    }

    // public Translation2d minus(Translation2d other) {
    //     return new Translation2d(x_ - other.x(), y_ - other.y());
    // }

    @Override
    public Translation2d unaryMinus() {
        return new Translation2d(super.unaryMinus());
        // return new Translation2d(-x_, -y_);
    }

    // public Translation2d times(double scalar) {
    //     return new Translation2d(x_ * scalar, y_ * scalar);
    // }

    /**
     * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation2d rotateBy(final Rotation2d rotation) {
        return new Translation2d(super.rotateBy(rotation));
        // return new Translation2d(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
    }

    public Rotation2d direction() {
        return new Rotation2d(getX(), getY());
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    // public Translation2d inverse() {
    //     return new Translation2d(-x_, -y_);
    // }

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

    // public boolean epsilonEquals(final edu.wpi.first.math.geometry.Translation2d other, double epsilon) {
    //     return Util.epsilonEquals(getX(), other.getX(), epsilon) && Util.epsilonEquals(getY(), other.getY(), epsilon);
    // }
    // @Override
    // public String toString() {
    //     final DecimalFormat format = new DecimalFormat("#0.000");
    //     return "(" + format.format(x_) + "," + format.format(y_) + ")";
    // }

    // @Override
    // public String toCSV() {
    //     final DecimalFormat format = new DecimalFormat("#0.000");
    //     return format.format(x_) + "," + format.format(y_);
    // }

    // public static double dot(final Translation2d a, final Translation2d b) {
    //     return a.x_ * b.x_ + a.y_ * b.y_;
    // }

    // public static Rotation2d getAngle(final Translation2d a, final Translation2d b) {
    //     double cos_angle = dot(a, b) / (a.norm() * b.norm());
    //     if (Double.isNaN(cos_angle)) {
    //         return new Rotation2d();
    //     }
    //     return Rotation2d.fromRadians(Math.acos(Util.limit(cos_angle, 1.0)));
    // }

    // public static double cross(final Translation2d a, final Translation2d b) {
    //     return a.x_ * b.y_ - a.y_ * b.x_;
    // }

    @Override
    public double distance(final Translation2d other) {
        return getDistance(other);
        // return unaryMinus().translateBy(other).norm();
    }

    @Override
    public Translation2d add(Translation2d other) {
        return new Translation2d(plus(other));
        // return this.translateBy(other);
    }

    // @Override
    // public boolean equals(final Object other) {
    //     if (!(other instanceof Translation2d)) {
    //         return false;
    //     }

    //     return distance((Translation2d) other) < Util.kEpsilon;
    // }

    public Translation2d getTranslation() {
        return this;
    }
}
