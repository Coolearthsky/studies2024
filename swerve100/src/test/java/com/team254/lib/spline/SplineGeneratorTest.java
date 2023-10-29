package com.team254.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.trajectory.TrajectoryPoint;
import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class SplineGeneratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // Create the test spline
        Pose2d p1 = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        Pose2d p2 = new Pose2d(new Translation2d(15, 10), new Rotation2d(1, -5));
        Spline s = new QuinticHermiteSpline(p1, p2);
        List<Rotation2dState> headings = List.of(GeometryUtil.fromDegrees(0), GeometryUtil.fromDegrees(90));

        List<TrajectoryPoint<Pose2dWithCurvature, Rotation2dState>> samples = SplineGenerator.parameterizeSpline(s,
                headings);

        double arclength = 0;
        Rotation2dState cur_heading = new Rotation2dState();
        Pose2dWithCurvature cur_pose = samples.get(0).state();
        for (TrajectoryPoint<Pose2dWithCurvature, Rotation2dState> point : samples) {
            Pose2dWithCurvature sample = point.state();
            final Twist2d t = GeometryUtil
                    .slog(GeometryUtil.transformBy(GeometryUtil.inverse(cur_pose.getPose()), sample.getPose().get()));
            arclength += t.dx;
            cur_pose = sample;
            cur_heading = point.heading();
        }

        assertEquals(cur_pose.getTranslation().getX(), 15.0, kTestEpsilon);
        assertEquals(cur_pose.getTranslation().getY(), 10.0, kTestEpsilon);
        assertEquals(cur_pose.getRotation().get().getDegrees(), -78.69006752597981, kTestEpsilon);
        assertEquals(arclength, 23.17291953186379, kTestEpsilon);
        assertEquals(cur_heading.get().getRadians(), headings.get(1).get().getRadians(), kTestEpsilon);
    }
}
