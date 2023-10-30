package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.trajectory.timing.TimedState;

/**
 * Represents a point and time on a 2d path with heading.
 */
public class TrajectoryPoint {
    private final TimedState<Pose2dWithCurvature> state_;
    private final TimedState<Rotation2dState> heading_;
    private final int index_;

    public TrajectoryPoint(final TimedState<Pose2dWithCurvature> state, TimedState<Rotation2dState> heading, int index) {
        state_ = state;
        heading_ = heading;
        index_ = index;
    }

    public TimedState<Pose2dWithCurvature> state() {
        return state_;
    }

    public TimedState<Rotation2dState> heading() {
        return heading_;
    }

    public int index() {
        return index_;
    }
}
