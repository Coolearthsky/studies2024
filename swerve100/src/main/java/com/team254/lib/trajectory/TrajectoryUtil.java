package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.spline.QuinticHermiteSpline;
import com.team254.lib.spline.Spline;
import com.team254.lib.spline.SplineGenerator;

import edu.wpi.first.math.geometry.Pose2d;

public class TrajectoryUtil {

    public static Trajectory<Pose2dWithCurvature, Rotation2dState> trajectoryFromWaypoints(
            final List<Pose2d> waypoints, final List<Rotation2dState> headings, double maxDx, double maxDy,
            double maxDTheta) {
        List<QuinticHermiteSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermiteSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        QuinticHermiteSpline.optimizeSpline(splines);
        return trajectoryFromSplinesAndHeadings(splines, headings, maxDx, maxDy, maxDTheta);
    }

    public static Trajectory<Pose2dWithCurvature, Rotation2dState> trajectoryFromSplinesAndHeadings(
            final List<? extends Spline> splines, final List<Rotation2dState> headings, double maxDx, double maxDy,
            double maxDTheta) {
        return new Trajectory<>(SplineGenerator.parameterizeSplines(splines, headings, maxDx, maxDy,
                maxDTheta));
    }

    private TrajectoryUtil() {
    }
}
