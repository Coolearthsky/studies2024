package com.team254.lib.geometry;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.spline.PoseWithCurvature;

public class Pose2dWithCurvature extends PoseWithCurvature {
    protected final double dcurvature_ds_;

    public Pose2dWithCurvature() {
        super();
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature) {
        this(pose, curvature, 0.0);
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature, double dcurvature_ds) {
        super(pose, curvature);
        dcurvature_ds_ = dcurvature_ds;
    }

    public final Pose2d getPose() {
        return poseMeters;
    }

    public double getCurvature() {
        return super.curvatureRadPerMeter;
    }

    public double getDCurvatureDs() {
        return dcurvature_ds_;
    }

    public Pose2dWithCurvature interpolate2(final Pose2dWithCurvature other, double x) {
        Pose2d interpolatedPose = getPose().interpolate(other.getPose(), x);
        double interpolatedCurvature = MathUtil.interpolate(getCurvature(), other.getCurvature(), x);
        double interpolatedCurvatureDs = MathUtil.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x);
        return new Pose2dWithCurvature(
                interpolatedPose,
                interpolatedCurvature,
                interpolatedCurvatureDs);
    }

    public double distance(final Pose2dWithCurvature other) {
        // this is not used
        return GeometryUtil.norm(
                GeometryUtil.slog(GeometryUtil.transformBy(GeometryUtil.inverse(getPose()), other.getPose())));
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithCurvature)) {
            return false;
        }
        Pose2dWithCurvature p2dwc = (Pose2dWithCurvature) other;
        return getPose().equals(p2dwc.getPose()) && Math.abs(getCurvature() - p2dwc.getCurvature()) <= 1e-12
                && Math.abs(getDCurvatureDs() - p2dwc.getDCurvatureDs()) <= 1e-12;
    }
}
