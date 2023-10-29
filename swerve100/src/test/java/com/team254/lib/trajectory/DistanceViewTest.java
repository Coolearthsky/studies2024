package com.team254.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DistanceViewTest {
    public static final double kTestEpsilon = 1e-12;

    @Test
    void test() {
        List<Pose2dWithCurvature> waypoints = Arrays.asList(
                new Pose2dWithCurvature(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()), 0),
                new Pose2dWithCurvature(new Pose2d(new Translation2d(24.0, 0.0), new Rotation2d()), 0),
                new Pose2dWithCurvature(new Pose2d(new Translation2d(36.0, 0.0), new Rotation2d()), 0),
                new Pose2dWithCurvature(new Pose2d(new Translation2d(36.0, 24.0), new Rotation2d()), 0),
                new Pose2dWithCurvature(new Pose2d(new Translation2d(60.0, 24.0), new Rotation2d()), 0));

        List<Rotation2dState> headings = Arrays.asList(
                GeometryUtil.fromDegrees(0),
                GeometryUtil.fromDegrees(30),
                GeometryUtil.fromDegrees(60),
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));

        // Create the reference trajectory (straight line motion between waypoints).
        Trajectory<Pose2dWithCurvature, Rotation2dState> trajectory = new Trajectory<>(waypoints, headings);
        final DistanceView distance_view = new DistanceView(trajectory);

        assertEquals(0.0, distance_view.first_interpolant(), kTestEpsilon);
        assertEquals(84.0, distance_view.last_interpolant(), kTestEpsilon);

        assertEquals(waypoints.get(0), distance_view.sample(0.0).state());
        assertEquals(waypoints.get(0).getPose().getTranslation().interpolate(waypoints.get(1).getPose().getTranslation(), 0.5),
                distance_view.sample(12.0).state().getPose().getTranslation());
        assertEquals(waypoints.get(3).getPose().getTranslation().interpolate(waypoints.get(4).getPose().getTranslation(), 0.5),
                distance_view.sample(72.0).state().getPose().getTranslation());
        assertEquals(headings.get(0), distance_view.sample(0.0).heading());
        assertEquals(headings.get(0).get().interpolate(headings.get(1).get(), 0.5),
                distance_view.sample(12).heading().get());
        assertEquals(headings.get(3).get().interpolate(headings.get(4).get(), 0.5),
                distance_view.sample(72.0).heading().get());
    }

}
