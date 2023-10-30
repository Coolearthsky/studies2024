package com.team254.lib.trajectory;


import com.team254.lib.trajectory.timing.TimedPose;
import com.team254.lib.trajectory.timing.TimedRotation;

/**
 * Represents a point and time on a 2d path with heading.
 */
public class TrajectoryPoint {
    private final TimedPose state_;
    private final TimedRotation heading_;
    private final int index_;

    public TrajectoryPoint(final TimedPose state, TimedRotation heading, int index) {
        state_ = state;
        heading_ = heading;
        index_ = index;
    }

    public TimedPose state() {
        return state_;
    }

    public TimedRotation heading() {
        return heading_;
    }

    public int index() {
        return index_;
    }
}
