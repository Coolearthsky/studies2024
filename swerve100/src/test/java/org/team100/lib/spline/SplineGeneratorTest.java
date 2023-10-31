package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.spline.PoseWithCurvature;

class SplineGeneratorTest {
    @Test
    void test() {
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
            final Twist2d twist = GeometryUtil.slog(
                    GeometryUtil.transformBy(
                            GeometryUtil.inverse(cur_pose.poseMeters), sample.poseMeters));
            arclength += twist.dx;
            cur_pose = sample;
            cur_heading = point.heading();
        }

        assertEquals(15.0, cur_pose.poseMeters.getTranslation().getX(), 0.001);
        assertEquals(10.0, cur_pose.poseMeters.getTranslation().getY(), 0.001);
        assertEquals(-78.690, cur_pose.poseMeters.getRotation().getDegrees(), 0.001);
        assertEquals(23.226, arclength, 0.001);
        assertEquals(cur_heading.getRadians(), headings.get(1).getRadians(), 1e-12);
    }
}
