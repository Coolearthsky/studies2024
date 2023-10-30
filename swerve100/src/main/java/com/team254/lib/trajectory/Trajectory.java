package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.trajectory.timing.TimedState;

/**
 * Represents a 2d path with heading and a schedule.
 */
public class Trajectory {
    protected final List<TrajectoryPoint> points_;

    public Trajectory(final List<TimedState<Pose2dWithCurvature>> states, final List<TimedState<Rotation2dState>> headings) {
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
