package org.team100.lib.trajectory;

import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimedRotation;

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
