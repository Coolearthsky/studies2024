package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimedRotation;

/**
 * Represents a 2d path with heading and a schedule.
 */
public class Trajectory {
    protected final List<TrajectoryPoint> points_;

    public Trajectory(final List<TimedPose> states, final List<TimedRotation> headings) {
        points_ = new ArrayList<>(states.size());
        for (int i = 0; i < states.size(); ++i) {
            points_.add(new TrajectoryPoint(states.get(i), headings.get(i), i));
        }
    }

    public boolean isEmpty() {
        return points_.isEmpty();
    }

    public int length() {
        return points_.size();
    }

    public TrajectoryPoint getLastPoint() {
        return points_.get(length() - 1);
    }

    public TrajectoryPoint getPoint(final int index) {
        return points_.get(index);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getPoint(i).state());
            builder.append(getPoint(i).heading());
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }
}
