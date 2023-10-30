package com.team254.lib.geometry;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pose2dState implements State<Pose2dState> {
    public final Pose2d pose2d;

    public Pose2d get() {
        return pose2d;
    }

    public Pose2dState(double x, double y, final Rotation2d rotation) {
        this(new Translation2d(x, y), rotation);
    }

    private Pose2dState(final Translation2d translation, final Rotation2d rotation) {
        pose2d = new Pose2d(translation, rotation);
    }

    public Pose2dState(final Pose2d other) {
        this(other.getTranslation(), other.getRotation());
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2dState interpolate2(final Pose2dState other, double x) {
        return new Pose2dState(get().interpolate(other.get(), x));
    }

    @Override
    public double distance(final Pose2dState other) {
        // this is never used.
        return GeometryUtil.norm(GeometryUtil.slog(GeometryUtil.transformBy(GeometryUtil.inverse(this), other.get())));
    }

    @Override
    public boolean equals(Object other) {
        if (!(other instanceof Pose2dState)) {
            return false;
        }
        return get().equals(((Pose2dState)other).get());
    }
}
