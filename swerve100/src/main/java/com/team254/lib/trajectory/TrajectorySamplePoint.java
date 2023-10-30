package com.team254.lib.trajectory;

import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimedRotation;

/** Represents a sample of a 2d path with heading and a schedule. */
public class TrajectorySamplePoint {
    protected final TimedPose state_;
    protected final TimedRotation heading_;
    protected final int index_floor_;
    protected final int index_ceil_;

    public TrajectorySamplePoint(final TimedPose state, final TimedRotation heading, int index_floor, int index_ceil) {
        state_ = state;
        heading_ = heading;
        index_floor_ = index_floor;
        index_ceil_ = index_ceil;
    }

    public TimedPose state() {
        return state_;
    }

    public TimedRotation heading() {
        return heading_;
    }

    public int index_floor() {
        return index_floor_;
    }

    public int index_ceil() {
        return index_ceil_;
    }
}
