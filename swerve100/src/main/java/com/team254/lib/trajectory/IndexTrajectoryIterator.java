package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;

public class IndexTrajectoryIterator {
    protected final IndexView view_;
    protected double progress_ = 0.0;
    protected TrajectorySamplePoint<Pose2dWithCurvature, Rotation2dState> current_sample_;

    public IndexTrajectoryIterator(final IndexView view) {
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

    public TrajectorySamplePoint<Pose2dWithCurvature, Rotation2dState> getSample() {
        return current_sample_;
    }

    public Pose2dWithCurvature getState() {
        return getSample().state();
    }

    public Rotation2dState getHeading() {
        return getSample().heading();
    }

    public TrajectorySamplePoint<Pose2dWithCurvature, Rotation2dState> advance(double additional_progress) {
        progress_ = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        current_sample_ = view_.sample(progress_);
        return current_sample_;
    }

    public TrajectorySamplePoint<Pose2dWithCurvature, Rotation2dState> preview(double additional_progress) {
        final double progress = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        return view_.sample(progress);
    }

    public Trajectory<Pose2dWithCurvature, Rotation2dState> trajectory() {
        return view_.trajectory();
    }
}
