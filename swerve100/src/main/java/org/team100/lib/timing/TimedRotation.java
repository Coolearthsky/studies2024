package org.team100.lib.timing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import java.text.DecimalFormat;

import org.team100.lib.geometry.GeometryUtil;

public class TimedRotation {
    protected final Rotation2d state_;
    protected double t_; // Time we achieve this state.
    protected double velocity_; // ds/dt
    protected double acceleration_; // d^2s/dt^2

    public TimedRotation(final Rotation2d state) {
        state_ = state;
    }

    public TimedRotation(final Rotation2d state, double t, double velocity, double acceleration) {
        state_ = state;
        t_ = t;
        velocity_ = velocity;
        acceleration_ = acceleration;
    }

    public Rotation2d state() {
        return state_;
    }

    public void set_t(double t) {
        t_ = t;
    }

    public double t() {
        return t_;
    }

    public void set_velocity(double velocity) {
        velocity_ = velocity;
    }

    public double velocity() {
        return velocity_;
    }

    public void set_acceleration(double acceleration) {
        acceleration_ = acceleration;
    }

    public double acceleration() {
        return acceleration_;
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return state().toString() + ", t: " + fmt.format(t()) + ", v: " + fmt.format(velocity()) + ", a: "
                + fmt.format(acceleration());
    }

    public TimedRotation interpolate2(TimedRotation other, double x) {
        final double new_t = MathUtil.interpolate(t(), other.t(), x);
        final double delta_t = new_t - t();
        if (delta_t < 0.0) {
            return other.interpolate2(this, 1.0 - x);
        }
        boolean reversing = velocity() < 0.0 || (Math.abs(velocity() - 0.0) <= 1e-12 && acceleration() < 0.0);
        final double new_v = velocity() + acceleration() * delta_t;
        final double new_s = (reversing ? -1.0 : 1.0)
                * (velocity() * delta_t + .5 * acceleration() * delta_t * delta_t);
        // System.out.println("x: " + x + " , new_t: " + new_t + ", new_s: " + new_s + "
        // , distance: " + state()
        // .distance(other.state()));
        return new TimedRotation(GeometryUtil.interpolate2(state(), other.state(), new_s / GeometryUtil.distance(state(), other.state())),
                new_t,
                new_v,
                acceleration());
    }

    // @Override
    // public TimedState<S> add(TimedState<S> other) {
    //     return new TimedState<>(this.state().add(other.state()));
    // }

    public double distance(TimedRotation other) {
        return GeometryUtil.distance(state(), other.state());
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof TimedRotation)) {
            System.out.println("wrong type");
            return false;
        }
        TimedRotation ts = (TimedRotation) other;
        boolean stateEqual = state().equals(ts.state());
        if (!stateEqual) {
            System.out.println("states not equal");
            return false;
        }
        boolean timeEqual = Math.abs(t() - ts.t()) <= 1e-12;
        if (!timeEqual) {
            System.out.println("time not equal");
            return false;
        }
        return stateEqual && timeEqual;
    }
}
