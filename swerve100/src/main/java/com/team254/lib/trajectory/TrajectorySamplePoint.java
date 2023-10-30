package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.trajectory.timing.TimedState;

/** Represents a sample of a 2d path with heading and a schedule. */
public class TrajectorySamplePoint {
    protected final TimedState<Pose2dWithCurvature> state_;
    protected final TimedState<Rotation2dState> heading_;
    protected final int index_floor_;
    protected final int index_ceil_;

    public TrajectorySamplePoint(final TimedState<Pose2dWithCurvature> state, final TimedState<Rotation2dState> heading, int index_floor, int index_ceil) {
        state_ = state;
        heading_ = heading;
        index_floor_ = index_floor;
        index_ceil_ = index_ceil;
    }

    public TimedState<Pose2dWithCurvature> state() {
        return state_;
    }

    public TimedState<Rotation2dState> heading() {
        return heading_;
    }

    public int index_floor() {
        return index_floor_;
    }

    public int index_ceil() {
        return index_ceil_;
    }
}
