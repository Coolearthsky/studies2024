package com.team254.lib.trajectory;

import com.team254.lib.trajectory.timing.TimedPose;
import com.team254.lib.trajectory.timing.TimedRotation;

/** 
 * Allows iterating over the schedule of a trajectory. 
 * */
public class TrajectoryTimeIterator {
    protected final TrajectoryTimeSampler view_;
    protected double progress_ = 0.0;
    protected TrajectorySamplePoint current_sample_;

    public TrajectoryTimeIterator() {
        view_=null;
    }

    public TrajectoryTimeIterator(final TrajectoryTimeSampler view) {
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

    public TrajectorySamplePoint getSample() {
        return current_sample_;
    }

    public TimedPose getState() {
        return getSample().state();
    }

    public TimedRotation getHeading() {
        return getSample().heading();
    }

    public TrajectorySamplePoint advance(double additional_progress) {
        progress_ = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        current_sample_ = view_.sample(progress_);
        return current_sample_;
    }

    public TrajectorySamplePoint preview(double additional_progress) {
        final double progress = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        return view_.sample(progress);
    }

    public Trajectory trajectory() {
        return view_.trajectory();
    }
}
