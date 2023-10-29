package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;
import com.team254.lib.trajectory.timing.TimedState;

public class TimedView<S extends State<S>, T extends State<T>> {
    protected final Trajectory<TimedState<S>, TimedState<T>> trajectory_;
    protected final double start_t_;
    protected final double end_t_;

    public TimedView(Trajectory<TimedState<S>, TimedState<T>> trajectory) {
        trajectory_ = trajectory;
        start_t_ = trajectory_.getPoint(0).state().t();
        end_t_ = trajectory_.getPoint(trajectory_.length() - 1).state().t();
    }

    public double first_interpolant() {
        return start_t_;
    }

    public double last_interpolant() {
        return end_t_;
    }

    public TrajectorySamplePoint<TimedState<S>, TimedState<T>> sample(double t) {
        if (t >= end_t_) {
            TrajectoryPoint<TimedState<S>, TimedState<T>> point = trajectory_.getPoint(trajectory_.length() - 1);
            return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
        }
        if (t <= start_t_) {
            TrajectoryPoint<TimedState<S>, TimedState<T>> point = trajectory_.getPoint(0);
            return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
        }
        for (int i = 1; i < trajectory_.length(); ++i) {
            final TrajectoryPoint<TimedState<S>, TimedState<T>> point = trajectory_.getPoint(i);
            if (point.state().t() >= t) {
                final TrajectoryPoint<TimedState<S>, TimedState<T>> prev_s = trajectory_.getPoint(i - 1);
                if (Math.abs(point.state().t() - prev_s.state().t()) <= 1e-12) {
                    return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
                }
                return new TrajectorySamplePoint<>(
                        prev_s.state().interpolate2(point.state(),
                                (t - prev_s.state().t()) / (point.state().t() - prev_s.state().t())),
                        prev_s.heading().interpolate2(point.heading(),
                                (t - prev_s.heading().t()) / (point.heading().t() - prev_s.heading().t())),
                        i - 1, i);
            }
        }
        throw new RuntimeException();
    }

    public Trajectory<TimedState<S>, TimedState<T>> trajectory() {
        return trajectory_;
    }
}
