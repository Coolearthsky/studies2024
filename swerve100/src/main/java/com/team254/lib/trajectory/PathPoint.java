package com.team254.lib.trajectory;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;

/**
 * Represents a point on a 2d path with heading.
 * 
 * There's no timing data here; for that, see TrajectoryPoint.
 */
public class PathPoint {
    private final PoseWithCurvature state_;
    private final Rotation2d heading_;
    private final int index_;

    public PathPoint(final PoseWithCurvature state, Rotation2d heading, int index) {
        state_ = state;
        heading_ = heading;
        index_ = index;
    }

    public PoseWithCurvature state() {
        return state_;
    }

    public Rotation2d heading() {
        return heading_;
    }

    public int index() {
        return index_;
    }
}
