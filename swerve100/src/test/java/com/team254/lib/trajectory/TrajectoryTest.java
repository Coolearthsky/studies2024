package com.team254.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.util.Util;

public class TrajectoryTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2dState> kWaypoints = Arrays.asList(
            new Translation2dState(0.0, 0.0),
            new Translation2dState(24.0, 0.0),
            new Translation2dState(36.0, 12.0),
            new Translation2dState(60.0, 12.0));

    List<Rotation2dState> kHeadings = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90),
            GeometryUtil.fromDegrees(180));

    @Test
    public void testConstruction() {
        // Empty constructor.
        Trajectory<Translation2dState, Rotation2dState> traj = new Trajectory<>();
        assertTrue(traj.isEmpty());
        assertEquals(0.0, traj.getIndexView().first_interpolant(), kTestEpsilon);
        assertEquals(0.0, traj.getIndexView().last_interpolant(), kTestEpsilon);
        assertEquals(0, traj.length());

        // Set states at construction time.
        traj = new Trajectory<>(kWaypoints, kHeadings);
        assertFalse(traj.isEmpty());
        assertEquals(0.0, traj.getIndexView().first_interpolant(), kTestEpsilon);
        assertEquals(3.0, traj.getIndexView().last_interpolant(), kTestEpsilon);
        assertEquals(4, traj.length());
    }

    @Test
    public void testStateAccessors() {
        Trajectory<Translation2dState, Rotation2dState> traj = new Trajectory<>(kWaypoints, kHeadings);

        assertEquals(kWaypoints.get(0), traj.getPoint(0).state());
        assertEquals(kWaypoints.get(1), traj.getPoint(1).state());
        assertEquals(kWaypoints.get(2), traj.getPoint(2).state());
        assertEquals(kWaypoints.get(3), traj.getPoint(3).state());

        assertEquals(kHeadings.get(0), traj.getPoint(0).heading());
        assertEquals(kHeadings.get(1), traj.getPoint(1).heading());
        assertEquals(kHeadings.get(2), traj.getPoint(2).heading());
        assertEquals(kHeadings.get(3), traj.getPoint(3).heading());

        assertEquals(kWaypoints.get(0), traj.getInterpolated(0.0).state());
        assertEquals(traj.getInterpolated(0.0).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.0).index_ceil(), 0);
        assertEquals(kWaypoints.get(1), traj.getInterpolated(1.0).state());
        assertEquals(traj.getInterpolated(1.0).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.0).index_ceil(), 1);
        assertEquals(kWaypoints.get(2), traj.getInterpolated(2.0).state());
        assertEquals(traj.getInterpolated(2.0).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.0).index_ceil(), 2);
        assertEquals(kWaypoints.get(3), traj.getInterpolated(3.0).state());
        assertEquals(traj.getInterpolated(3.0).index_floor(), 3);
        assertEquals(traj.getInterpolated(3.0).index_ceil(), 3);

        assertEquals(kHeadings.get(0), traj.getInterpolated(0.0).heading());
        assertEquals(traj.getInterpolated(0.0).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.0).index_ceil(), 0);
        assertEquals(kHeadings.get(1), traj.getInterpolated(1.0).heading());
        assertEquals(traj.getInterpolated(1.0).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.0).index_ceil(), 1);
        assertEquals(kHeadings.get(2), traj.getInterpolated(2.0).heading());
        assertEquals(traj.getInterpolated(2.0).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.0).index_ceil(), 2);
        assertEquals(kHeadings.get(3), traj.getInterpolated(3.0).heading());
        assertEquals(traj.getInterpolated(3.0).index_floor(), 3);
        assertEquals(traj.getInterpolated(3.0).index_ceil(), 3);

        assertEquals(kWaypoints.get(0).get().interpolate(kWaypoints.get(1).get(), .25), traj.getInterpolated(0.25).state().get());
        assertEquals(traj.getInterpolated(0.25).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.25).index_ceil(), 1);
        assertEquals(kWaypoints.get(1).get().interpolate(kWaypoints.get(2).get(), .5), traj.getInterpolated(1.5).state().get());
        assertEquals(traj.getInterpolated(1.5).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.5).index_ceil(), 2);
        assertEquals(kWaypoints.get(2).get().interpolate(kWaypoints.get(3).get(), .75), traj.getInterpolated(2.75).state().get());
        assertEquals(traj.getInterpolated(2.75).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.75).index_ceil(), 3);

        assertEquals(kHeadings.get(0).get().interpolate(kHeadings.get(1).get(), .25), traj.getInterpolated(0.25).heading().get());
        assertEquals(traj.getInterpolated(0.25).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.25).index_ceil(), 1);
        assertEquals(kHeadings.get(1).get().interpolate(kHeadings.get(2).get(), .5), traj.getInterpolated(1.5).heading().get());
        assertEquals(traj.getInterpolated(1.5).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.5).index_ceil(), 2);
        assertEquals(kHeadings.get(2).get().interpolate(kHeadings.get(3).get(), .75), traj.getInterpolated(2.75).heading().get());
        assertEquals(traj.getInterpolated(2.75).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.75).index_ceil(), 3);

        Trajectory<Translation2dState, Rotation2dState>.IndexView index_view = traj.getIndexView();
        assertEquals(kWaypoints.get(0).get().interpolate(kWaypoints.get(1).get(), .25), index_view.sample(0.25).state().get());
        assertEquals(kWaypoints.get(1).get().interpolate(kWaypoints.get(2).get(), .5), index_view.sample(1.5).state().get());
        assertEquals(kWaypoints.get(2).get().interpolate(kWaypoints.get(3).get(), .75), index_view.sample(2.75).state().get());
        assertEquals(kHeadings.get(0).get().interpolate(kHeadings.get(1).get(), .25), index_view.sample(0.25).heading().get());
        assertEquals(kHeadings.get(1).get().interpolate(kHeadings.get(2).get(), .5), index_view.sample(1.5).heading().get());
        assertEquals(kHeadings.get(2).get().interpolate(kHeadings.get(3).get(), .75), index_view.sample(2.75).heading().get());
    }
}
