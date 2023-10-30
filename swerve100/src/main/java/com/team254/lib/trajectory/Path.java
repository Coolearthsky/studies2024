package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;

/**
 * Represents a 2d path with heading.
 * 
 * There's no timing information here.  For that, see Trajectory.
 */
public class Path {
    protected final List<PathPoint> points_;

    public Path(final List<Pose2dWithCurvature> states, final List<Rotation2dState> headings) {
        points_ = new ArrayList<>(states.size());
        for (int i = 0; i < states.size(); ++i) {
            points_.add(new PathPoint(states.get(i), headings.get(i), i));
        }
    }

    public Path(final List<PathPoint> points) {
        // this renumbers the points.
        points_ = new ArrayList<>(points.size());
        for (int i = 0; i < points.size(); i++) {
            points_.add(new PathPoint(points.get(i).state(), points.get(i).heading(), i));
        }
    }

    public boolean isEmpty() {
        return points_.isEmpty();
    }

    public int length() {
        return points_.size();
    }

    public PathPoint getPoint(final int index) {
        return points_.get(index);
    }

    public PathSamplePoint getInterpolated(final double index) {
        if (isEmpty()) {
            return null;
        } else if (index <= 0.0) {
            PathPoint point = getPoint(0);
            return new PathSamplePoint(point.state(), point.heading(), point.index(), point.index());
        } else if (index >= length() - 1) {
            PathPoint point = getPoint(length() - 1);
            return new PathSamplePoint(point.state(), point.heading(), point.index(), point.index());
        }
        final int i = (int) Math.floor(index);
        final double frac = index - i;
        if (frac <= Double.MIN_VALUE) {
            PathPoint point = getPoint(i);
            return new PathSamplePoint(point.state(), point.heading(), point.index(), point.index());
        } else if (frac >= 1.0 - Double.MIN_VALUE) {
            PathPoint point = getPoint(i + 1);
            return new PathSamplePoint(point.state(), point.heading(), point.index(), point.index());
        } else {
            return new PathSamplePoint(
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
