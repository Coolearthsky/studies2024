package com.team254.lib.geometry;

import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;

public class Pose2dWithCurvature extends PoseWithCurvature implements State<Pose2dWithCurvature> {
    protected final double dcurvature_ds_;

    public Pose2dWithCurvature() {
        super();
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithCurvature(final Pose2dState pose, double curvature) {
        this(pose, curvature, 0.0);
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature) {
        this(pose, curvature, 0.0);
    }

    public Pose2dWithCurvature(final Pose2dState pose, double curvature, double dcurvature_ds) {
        super(pose.get(), curvature);
        dcurvature_ds_ = dcurvature_ds;
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature, double dcurvature_ds) {
        super(pose, curvature);
        dcurvature_ds_ = dcurvature_ds;
    }

    public final Pose2dState getPose() {
        return new Pose2dState(poseMeters);
    }

    public Pose2dWithCurvature transformBy(Pose2dState transform) {
        return new Pose2dWithCurvature(GeometryUtil.transformBy(getPose().get(), transform.get()), getCurvature(), getDCurvatureDs());
    }

    public double getCurvature() {
        return super.curvatureRadPerMeter;
    }

    public double getDCurvatureDs() {
        return dcurvature_ds_;
    }

    public final Translation2d getTranslation() {
        return getPose().get().getTranslation();
    }

    public final Rotation2dState getRotation() {
        return new Rotation2dState(getPose().get().getRotation());
    }

    @Override
    public Pose2dWithCurvature interpolate2(final Pose2dWithCurvature other, double x) {
        return new Pose2dWithCurvature(getPose().interpolate2(other.getPose(), x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    @Override
    public double distance(final Pose2dWithCurvature other) {
        return getPose().distance(other.getPose());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithCurvature)) {
            return false;
        }
        Pose2dWithCurvature p2dwc = (Pose2dWithCurvature) other;
        return getPose().equals(p2dwc.getPose()) && Util.epsilonEquals(getCurvature(), p2dwc.getCurvature()) && Util.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    // public Pose2dWithCurvature rotateBy(Rotation2dState other) {
    //     return new Pose2dWithCurvature(getPose().get().rotateBy(other.get()), getCurvature(), getDCurvatureDs());
    // }

    // @Override
    // public Pose2dWithCurvature add(Pose2dWithCurvature other) {
    //     return this.transformBy(new Pose2dState(other.poseMeters));   // todo make work
    // }
}
