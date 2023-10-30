package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;

/**
 * Represents a sample of a 2d path with heading.
 */
public class PathSamplePoint {
    private final Pose2dWithCurvature state_;
    private final Rotation2dState heading_;
    private final int index_floor_;
    private final int index_ceil_;

    public PathSamplePoint(
            final Pose2dWithCurvature state,
            final Rotation2dState heading,
            int index_floor,
            int index_ceil) {
        state_ = state;
        heading_ = heading;
        index_floor_ = index_floor;
        index_ceil_ = index_ceil;
    }

    public Pose2dWithCurvature state() {
        return state_;
    }

    public Rotation2dState heading() {
        return heading_;
    }

    public int index_floor() {
        return index_floor_;
    }

    public int index_ceil() {
        return index_ceil_;
    }
}
