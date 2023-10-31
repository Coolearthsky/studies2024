package org.team100.lib.path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;

/**
 * Represents a sample of a 2d path with heading.
 */
public class PathSamplePoint {
    private final PoseWithCurvature state_;
    private final Rotation2d heading_;
    private final int index_floor_;
    private final int index_ceil_;

    public PathSamplePoint(
            final PoseWithCurvature state,
            final Rotation2d heading,
            int index_floor,
            int index_ceil) {
        state_ = state;
        heading_ = heading;
        index_floor_ = index_floor;
        index_ceil_ = index_ceil;
    }

    public PoseWithCurvature state() {
        return state_;
    }

    public Rotation2d heading() {
        return heading_;
    }

    public int index_floor() {
        return index_floor_;
    }

    public int index_ceil() {
        return index_ceil_;
    }
}
