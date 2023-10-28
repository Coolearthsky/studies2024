package com.team254.lib.spline;

import com.team254.lib.geometry.*;
import com.team254.lib.trajectory.TrajectoryPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;

public class SplineGenerator {
    private static final double kMaxDX = 2.0; //inches
    private static final double kMaxDY = 0.05; //inches
    private static final double kMaxDTheta = 0.1; //radians!
    private static final int kMinSampleSize = 1;

    /**
     * Converts a spline into a list of Twist2d's.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of Pose2dWithCurvature that approximates the original spline
     */
    public static List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> parameterizeSpline(Spline s, List<? extends Rotation2dState> headings, double maxDx, double maxDy, double
            maxDTheta, double t0, double t1) {
        List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> rv = new ArrayList<>();
        rv.add(new TrajectoryPoint<>(s.getPose2dWithCurvature(0.0), headings.get(0), 0));
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt / kMinSampleSize) {
            getSegmentArc(s, headings, rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta, dt);
        }
        return rv;
    }

    /**
     * Convenience function to parametrize a spline from t 0 to 1
     */
    public static List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> parameterizeSpline(Spline s, List<? extends Rotation2dState> headings) {
        return parameterizeSpline(s, headings, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
    }

    public static List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> parameterizeSpline(Spline s, List<? extends Rotation2dState> headings, double maxDx, double maxDy, double maxDTheta) {
        return parameterizeSpline(s, headings, maxDx, maxDy, maxDTheta, 0.0, 1.0);
    }

    public static List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> parameterizeSplines(List<Spline> splines, List<? extends Rotation2dState> headings) {
        return parameterizeSplines(splines, headings, kMaxDX, kMaxDY, kMaxDTheta);
    }

    public static List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> parameterizeSplines(List<? extends Spline> splines, List<? extends Rotation2dState> headings, double maxDx, double maxDy,
                                                                double maxDTheta) {
        List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> rv = new ArrayList<>();
        if (splines.isEmpty()) return rv;
        rv.add(new TrajectoryPoint<>(splines.get(0).getPose2dWithCurvature(0.0), headings.get(0).getRotation(), 0));
        for (int i = 0; i < splines.size(); i++) {
            Spline s = splines.get(i);
            List<Rotation2dState> spline_rots = new ArrayList<>();
            spline_rots.add(headings.get(i));
            spline_rots.add(headings.get(i+1));

            List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> samples = parameterizeSpline(s, spline_rots, maxDx, maxDy, maxDTheta);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    private static void getSegmentArc(Spline s, List<? extends Rotation2dState> headings, List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> rv,  double t0, double t1, double maxDx,
                                      double maxDy,
                                      double maxDTheta, double totalTime) {
        Translation2d p0 = s.getPoint(t0).get();
        Translation2d p1 = s.getPoint(t1).get();
        Rotation2d r0 = s.getHeading(t0).get();
        Rotation2d r1 = s.getHeading(t1).get();
        Pose2d transformation = new Pose2d(p1.minus(p0).rotateBy(r0.unaryMinus()), r1.rotateBy(r0.unaryMinus()));
        Twist2dWrapper twist = GeometryUtil.slog(transformation);

        if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta) {
            getSegmentArc(s, headings, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta, totalTime);
            getSegmentArc(s, headings, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta, totalTime);
        } else {
            // Interpolate heading
            Rotation2d diff = headings.get(1).rotateBy(headings.get(0).get().unaryMinus()).get();
            if (diff.getRadians() > Math.PI) {
                diff = diff.unaryMinus().rotateBy(Rotation2d.fromRadians( Math.PI));
            }
            Rotation2dState interpolated_heading = headings.get(0).rotateBy(diff.times(t1 / totalTime));

            rv.add(new TrajectoryPoint<>(s.getPose2dWithCurvature(t1), interpolated_heading, rv.size()-1));
        }
    }

}
