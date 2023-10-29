package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;
import com.team254.lib.trajectory.timing.TimedState;

public class TimedTrajectoryIterator<S extends State<S>, T extends State<T>> {
    protected final TimedView<S, T> view_;
    protected double progress_ = 0.0;
    protected TrajectorySamplePoint<TimedState<S>, TimedState<T>> current_sample_;

    public TimedTrajectoryIterator() {
        view_=null;
    }

    public TimedTrajectoryIterator(final TimedView<S, T> view) {
        view_ = view;

        // No effect if view is empty.
        current_sample_ = view_.sample(view_.first_interpolant());
        progress_ = view_.first_interpolant();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress_;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, view_.last_interpolant() - progress_);
    }

    public TrajectorySamplePoint<TimedState<S>, TimedState<T>> getSample() {
        return current_sample_;
    }

    public TimedState<S> getState() {
        return getSample().state();
    }

    public TimedState<T> getHeading() {
        return getSample().heading();
    }

    public TrajectorySamplePoint<TimedState<S>, TimedState<T>> advance(double additional_progress) {
        progress_ = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        current_sample_ = view_.sample(progress_);
        return current_sample_;
    }

    public TrajectorySamplePoint<TimedState<S>, TimedState<T>> preview(double additional_progress) {
        final double progress = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        return view_.sample(progress);
    }

    public Trajectory<TimedState<S>, TimedState<T>> trajectory() {
        return view_.trajectory();
    }
}
