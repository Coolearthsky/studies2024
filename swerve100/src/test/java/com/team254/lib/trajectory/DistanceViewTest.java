package com.team254.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.util.Util;

public class DistanceViewTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // Specify desired waypoints.
        List<Translation2dState> waypoints = Arrays.asList(
                new Translation2dState(0.0, 0.0),
                new Translation2dState(24.0, 0.0),
                new Translation2dState(36.0, 0.0),
                new Translation2dState(36.0, 24.0),
                new Translation2dState(60.0, 24.0));
        List<Rotation2dState> headings = Arrays.asList(
                GeometryUtil.fromDegrees(0),
                GeometryUtil.fromDegrees(30),
                GeometryUtil.fromDegrees(60),
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));

        // Create the reference trajectory (straight line motion between waypoints).
        Trajectory<Translation2dState, Rotation2dState> trajectory = new Trajectory<>(waypoints, headings);
        final DistanceView<Translation2dState, Rotation2dState> distance_view = new DistanceView<>(trajectory);

        assertEquals(0.0, distance_view.first_interpolant(), kTestEpsilon);
        assertEquals(84.0, distance_view.last_interpolant(), kTestEpsilon);

        assertEquals(waypoints.get(0), distance_view.sample(0.0).state());
        assertEquals(waypoints.get(0).get().interpolate(waypoints.get(1).get(), 0.5), distance_view.sample(12.0).state().get());
        assertEquals(waypoints.get(3).get().interpolate(waypoints.get(4).get(), 0.5), distance_view.sample(72.0).state().get());
        assertEquals(headings.get(0), distance_view.sample(0.0).heading());
        assertEquals(headings.get(0).get().interpolate(headings.get(1).get(), 0.5), distance_view.sample(12).heading().get());
        assertEquals(headings.get(3).get().interpolate(headings.get(4).get(), 0.5), distance_view.sample(72.0).heading().get());
    }

}
