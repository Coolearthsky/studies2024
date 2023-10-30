package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;

/**
 * Represents a point on a 2d path with heading.
 * 
 * There's no timing data here; for that, see TrajectoryPoint.
 */
public class PathPoint {
    private final Pose2dWithCurvature state_;
    private final Rotation2dState heading_;
    private final int index_;

    public PathPoint(final Pose2dWithCurvature state, Rotation2dState heading, int index) {
        state_ = state;
        heading_ = heading;
        index_ = index;
    }

    public Pose2dWithCurvature state() {
        return state_;
    }

    public Rotation2dState heading() {
        return heading_;
    }

    public int index() {
        return index_;
    }
}
