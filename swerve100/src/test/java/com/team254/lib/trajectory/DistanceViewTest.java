package com.team254.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

public class DistanceViewTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // Specify desired waypoints.
        List<Translation2d> waypoints = Arrays.asList(
                new Translation2d(0.0, 0.0),
                new Translation2d(24.0, 0.0),
                new Translation2d(36.0, 0.0),
                new Translation2d(36.0, 24.0),
                new Translation2d(60.0, 24.0));
        List<Rotation2d> headings = Arrays.asList(
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(30),
                Rotation2d.fromDegrees(60),
                Rotation2d.fromDegrees(90),
                Rotation2d.fromDegrees(180));

        // Create the reference trajectory (straight line motion between waypoints).
        Trajectory<Translation2d, Rotation2d> trajectory = new Trajectory<>(waypoints, headings);
        final DistanceView<Translation2d, Rotation2d> distance_view = new DistanceView<>(trajectory);

        assertEquals(0.0, distance_view.first_interpolant(), kTestEpsilon);
        assertEquals(84.0, distance_view.last_interpolant(), kTestEpsilon);

        assertEquals(waypoints.get(0), distance_view.sample(0.0).state());
        assertEquals(waypoints.get(0).interpolate(waypoints.get(1), 0.5), distance_view.sample(12.0).state());
        assertEquals(waypoints.get(3).interpolate(waypoints.get(4), 0.5), distance_view.sample(72.0).state());
        assertEquals(headings.get(0), distance_view.sample(0.0).heading());
        assertEquals(headings.get(0).interpolate(headings.get(1), 0.5), distance_view.sample(12).heading());
        assertEquals(headings.get(3).interpolate(headings.get(4), 0.5), distance_view.sample(72.0).heading());
    }

}
