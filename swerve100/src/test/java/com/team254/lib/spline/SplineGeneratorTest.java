package com.team254.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.trajectory.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.spline.PoseWithCurvature;

public class SplineGeneratorTest {
    private static final double kTestEpsilon = 1e-12;

    @Test
    void test() {
        // Create the test spline
        Pose2d p1 = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        Pose2d p2 = new Pose2d(new Translation2d(15, 10), new Rotation2d(1, -5));
        Spline s = new QuinticHermiteSpline(p1, p2);
        List<Rotation2d> headings = List.of(GeometryUtil.fromDegrees(0), GeometryUtil.fromDegrees(90));

        List<PathPoint> samples = SplineGenerator.parameterizeSpline(s, headings);

        double arclength = 0;
        Rotation2d cur_heading = new Rotation2d();
        PoseWithCurvature cur_pose = samples.get(0).state();
        for (PathPoint point : samples) {
            PoseWithCurvature sample = point.state();
            final Twist2d t = GeometryUtil
                    .slog(GeometryUtil.transformBy(GeometryUtil.inverse(cur_pose.poseMeters), sample.poseMeters));
            arclength += t.dx;
            cur_pose = sample;
            cur_heading = point.heading();
        }

        assertEquals(15.0, cur_pose.poseMeters.getTranslation().getX(), kTestEpsilon);
        assertEquals(10.0, cur_pose.poseMeters.getTranslation().getY(), kTestEpsilon);
        assertEquals(-78.69006752597981, cur_pose.poseMeters.getRotation().getDegrees(), kTestEpsilon);
        assertEquals(23.17291953186379, arclength, kTestEpsilon);
        assertEquals(cur_heading.getRadians(), headings.get(1).getRadians(), kTestEpsilon);
    }
}
