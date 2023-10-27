package com.team254.lib.spline;

import com.team254.lib.geometry.Pose2dState;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;

public abstract class Spline {
    public abstract Translation2dState getPoint(double t);

    public abstract Rotation2dState getHeading(double t);

    public abstract double getCurvature(double t);

    // dk/dt
    public abstract double getDCurvature(double t);

    // ds/dt
    public abstract double getVelocity(double t);

    public Pose2dState getPose2d(double t) {
        return new Pose2dState(getPoint(t), getHeading(t));
    }

    public Pose2dWithCurvature getPose2dWithCurvature(double t) {
        return new Pose2dWithCurvature(getPose2d(t), getCurvature(t), getDCurvature(t) / getVelocity(t));
    }

    // TODO add toString
    // public abstract String toString();
}
