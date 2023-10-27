package com.team254.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.util.Util;

public class TrajectoryIteratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2dState> kWaypoints = Arrays.asList(
            new Translation2dState(0.0, 0.0),
            new Translation2dState(24.0, 0.0),
            new Translation2dState(36.0, 12.0),
            new Translation2dState(60.0, 12.0));

    List<Rotation2dState> kHeadings = Arrays.asList(
            Rotation2dState.fromDegrees(0),
            Rotation2dState.fromDegrees(30),
            Rotation2dState.fromDegrees(60),
            Rotation2dState.fromDegrees(90),
            Rotation2dState.fromDegrees(180));

    @Test
    public void test() {
        Trajectory<Translation2dState, Rotation2dState> traj = new Trajectory<>(kWaypoints, kHeadings);
        TrajectoryIterator<Translation2dState, Rotation2dState> iterator = new TrajectoryIterator<>(traj.getIndexView());

        // Initial conditions.
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertEquals(kWaypoints.get(0), iterator.getState());
        assertEquals(kHeadings.get(0), iterator.getHeading());
        assertFalse(iterator.isDone());

        // Advance forward.
        assertEquals(kWaypoints.get(0).get().interpolate(kWaypoints.get(1).get(), 0.5), iterator.preview(0.5).state().get());
        assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.5), iterator.preview(0.5).heading());
        TrajectorySamplePoint<Translation2dState, Rotation2dState> newPoint = iterator.advance(0.5);
        assertEquals(kWaypoints.get(0).get().interpolate(kWaypoints.get(1).get(), 0.5), newPoint.state().get());
        assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.5), newPoint.heading());
        assertEquals(0.5, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.5, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance backwards.
        assertEquals(kWaypoints.get(0).get().interpolate(kWaypoints.get(1).get(), 0.25), iterator.preview(-0.25).state().get());
        assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.25), iterator.preview(-0.25).heading());
        newPoint = iterator.advance(-0.25);
        assertEquals(kWaypoints.get(0).get().interpolate(kWaypoints.get(1).get(), 0.25), newPoint.state().get());
        assertEquals(kHeadings.get(0).interpolate(kHeadings.get(1), 0.25), newPoint.heading());
        assertEquals(0.25, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.75, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance past end.
        assertEquals(kWaypoints.get(3), iterator.preview(5.0).state());
        assertEquals(kHeadings.get(3), iterator.preview(5.0).heading());
        newPoint = iterator.advance(5.0);
        assertEquals(kWaypoints.get(3), newPoint.state());
        assertEquals(kHeadings.get(3), newPoint.heading());
        assertEquals(3.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(0.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertTrue(iterator.isDone());

        // Advance past beginning.
        assertEquals(kWaypoints.get(0), iterator.preview(-5.0).state());
        assertEquals(kHeadings.get(0), iterator.preview(-5.0).heading());
        newPoint = iterator.advance(-5.0);
        assertEquals(kWaypoints.get(0), newPoint.state());
        assertEquals(kHeadings.get(0), newPoint.heading());
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());
    }

}
