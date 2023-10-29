package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;

public class IndexView {
    private final Trajectory<Pose2dWithCurvature, Rotation2dState> trajectory;

    public IndexView(Trajectory<Pose2dWithCurvature, Rotation2dState> trajectory) {
        this.trajectory = trajectory;
    }

    public TrajectorySamplePoint<Pose2dWithCurvature, Rotation2dState> sample(double index) {
        return trajectory.getInterpolated(index);
    }

    public double last_interpolant() {
        return Math.max(0.0, trajectory.length() - 1);
    }

    public double first_interpolant() {
        return 0.0;
    }

    public Trajectory<Pose2dWithCurvature, Rotation2dState> trajectory() {
        return trajectory;
    }
}