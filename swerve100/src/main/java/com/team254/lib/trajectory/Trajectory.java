package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.State;

public class Trajectory<S extends State<S>, T extends State<T>> {
    protected final List<TrajectoryPoint<S, T>> points_;

    public Trajectory(final List<S> states, final List<T> headings) {
        points_ = new ArrayList<>(states.size());
        for (int i = 0; i < states.size(); ++i) {
            points_.add(new TrajectoryPoint<>(states.get(i), headings.get(i), i));
        }
    }

    public Trajectory(final List<TrajectoryPoint<S, T>> points) {
        // this renumbers the points.
        points_ = new ArrayList<>(points.size());
        for (int i = 0; i < points.size(); i++) {
            points_.add(new TrajectoryPoint<>(points.get(i).state(), points.get(i).heading(), i));
        }
    }

    public boolean isEmpty() {
        return points_.isEmpty();
    }

    public int length() {
        return points_.size();
    }

    public TrajectoryPoint<S, T> getLastPoint() {
        return points_.get(length() - 1);
    }

    public TrajectoryPoint<S, T> getPoint(final int index) {
        return points_.get(index);
    }

    public TrajectorySamplePoint<S, T> getInterpolated(final double index) {
        if (isEmpty()) {
            return null;
        } else if (index <= 0.0) {
            TrajectoryPoint<S, T> point = getPoint(0);
            return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
        } else if (index >= length() - 1) {
            TrajectoryPoint<S, T> point = getPoint(length() - 1);
            return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
        }
        final int i = (int) Math.floor(index);
        final double frac = index - i;
        if (frac <= Double.MIN_VALUE) {
            TrajectoryPoint<S, T> point = getPoint(i);
            return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
        } else if (frac >= 1.0 - Double.MIN_VALUE) {
            TrajectoryPoint<S, T> point = getPoint(i + 1);
            return new TrajectorySamplePoint<>(point.state(), point.heading(), point.index(), point.index());
        } else {
            return new TrajectorySamplePoint<>(
                    getPoint(i).state().interpolate2(getPoint(i + 1).state(), frac),
                    getPoint(i).heading().interpolate2(getPoint(i + 1).heading(), frac),
                    i,
                    i + 1);
        }
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
