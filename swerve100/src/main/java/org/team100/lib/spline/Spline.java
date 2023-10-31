package org.team100.lib.spline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;

public abstract class Spline {
    public abstract Translation2d getPoint(double t);

    public abstract Rotation2d getHeading(double t);

    public abstract double getCurvature(double t);

    // dk/dt
    public abstract double getDCurvature(double t);

    // ds/dt
    public abstract double getVelocity(double t);

    public Pose2d getPose2d(double t) {
        return new Pose2d(getPoint(t), getHeading(t));
    }

    // this used to also calculate the rate of change of curvature, presumably so it
    // would be useful for centripetal acceleration? anyway i removed it.
    public PoseWithCurvature getPose2dWithCurvature(double t) {
        return new PoseWithCurvature(getPose2d(t), getCurvature(t));
        // return new Pose2dWithCurvature(getPose2d(t), getCurvature(t),
        // getDCurvature(t) / getVelocity(t));

    }

}
