package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;

public class DistanceView<S extends State<S>, T extends State<T>> {
    protected final Trajectory<S, T> trajectory_;
    protected final double[] distances_;

    public DistanceView(final Trajectory<S, T> trajectory) {
        trajectory_ = trajectory;
        distances_ = new double[trajectory_.length()];
        distances_[0] = 0.0;
        for (int i = 1; i < trajectory_.length(); ++i) {
            distances_[i] = distances_[i - 1]
                    + trajectory_.getPoint(i - 1).state().distance(trajectory_.getPoint(i).state());
        }
    }

    public TrajectorySamplePoint<S, T> sample(double distance) {
        if (distance >= last_interpolant()) {
            TrajectoryPoint<S, T> point = trajectory_.getPoint(trajectory_.length() - 1);
            return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
        }
        if (distance <= 0.0) {
            TrajectoryPoint<S, T> point = trajectory_.getPoint(0);
            return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
        }
        for (int i = 1; i < distances_.length; ++i) {
            final TrajectoryPoint<S, T> point = trajectory_.getPoint(i);
            if (distances_[i] >= distance) {
                final TrajectoryPoint<S, T> prev_s = trajectory_.getPoint(i - 1);
                if (Math.abs(distances_[i] - distances_[i - 1]) <= 1e-12) {
                    return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
                } else {
                    return new TrajectorySamplePoint<>(
                            prev_s.state().interpolate2(point.state(),
                                    (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])),
                            prev_s.heading().interpolate2(point.heading(),
                                    (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])),
                            i - 1, i);
                }
            }
        }
        throw new RuntimeException();
    }

    public double last_interpolant() {
        return distances_[distances_.length - 1];
    }

    public double first_interpolant() {
        return 0.0;
    }

    public Trajectory<S, T> trajectory() {
        return trajectory_;
    }
}
