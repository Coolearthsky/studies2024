package com.team254.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryTest {
    public static final double kTestEpsilon = 1e-12;


            public static final List<Pose2dWithCurvature> kWaypoints = Arrays.asList(
                new Pose2dWithCurvature(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()), 0),
                new Pose2dWithCurvature(new Pose2d(new Translation2d(24.0, 0.0), new Rotation2d()), 0),
                new Pose2dWithCurvature(new Pose2d(new Translation2d(36.0, 12.0), new Rotation2d()), 0),
                new Pose2dWithCurvature(new Pose2d(new Translation2d(60.0, 12.0), new Rotation2d()), 0));

    List<Rotation2dState> kHeadings = Arrays.asList(
            GeometryUtil.fromDegrees(0),
            GeometryUtil.fromDegrees(30),
            GeometryUtil.fromDegrees(60),
            GeometryUtil.fromDegrees(90),
            GeometryUtil.fromDegrees(180));

    @Test
    public void testConstruction() {
        Path traj = new Path(kWaypoints, kHeadings);
        assertFalse(traj.isEmpty());
        assertEquals(4, traj.length());
    }

    @Test
    public void testStateAccessors() {
        Path traj = new Path(kWaypoints, kHeadings);

        assertEquals(kWaypoints.get(0), traj.getPoint(0).state());
        assertEquals(kWaypoints.get(1), traj.getPoint(1).state());
        assertEquals(kWaypoints.get(2), traj.getPoint(2).state());
        assertEquals(kWaypoints.get(3), traj.getPoint(3).state());

        assertEquals(kHeadings.get(0), traj.getPoint(0).heading());
        assertEquals(kHeadings.get(1), traj.getPoint(1).heading());
        assertEquals(kHeadings.get(2), traj.getPoint(2).heading());
        assertEquals(kHeadings.get(3), traj.getPoint(3).heading());

        assertEquals(kWaypoints.get(0), traj.getInterpolated(0.0).state());
        assertEquals(0, traj.getInterpolated(0.0).index_floor());
        assertEquals(0, traj.getInterpolated(0.0).index_ceil());
        assertEquals(kWaypoints.get(1), traj.getInterpolated(1.0).state());
        assertEquals(1, traj.getInterpolated(1.0).index_floor());
        assertEquals(1, traj.getInterpolated(1.0).index_ceil());
        assertEquals(kWaypoints.get(2), traj.getInterpolated(2.0).state());
        assertEquals(2, traj.getInterpolated(2.0).index_floor());
        assertEquals(2, traj.getInterpolated(2.0).index_ceil());
        assertEquals(kWaypoints.get(3), traj.getInterpolated(3.0).state());
        assertEquals(3, traj.getInterpolated(3.0).index_floor());
        assertEquals(3, traj.getInterpolated(3.0).index_ceil());

        assertEquals(kHeadings.get(0), traj.getInterpolated(0.0).heading());
        assertEquals(0, traj.getInterpolated(0.0).index_floor());
        assertEquals(0, traj.getInterpolated(0.0).index_ceil());
        assertEquals(kHeadings.get(1), traj.getInterpolated(1.0).heading());
        assertEquals(1, traj.getInterpolated(1.0).index_floor());
        assertEquals(1, traj.getInterpolated(1.0).index_ceil());
        assertEquals(kHeadings.get(2), traj.getInterpolated(2.0).heading());
        assertEquals(2, traj.getInterpolated(2.0).index_floor());
        assertEquals(2, traj.getInterpolated(2.0).index_ceil());
        assertEquals(kHeadings.get(3), traj.getInterpolated(3.0).heading());
        assertEquals(3, traj.getInterpolated(3.0).index_floor());
        assertEquals(3, traj.getInterpolated(3.0).index_ceil());

        assertEquals(kWaypoints.get(0).getPose().getTranslation().interpolate(kWaypoints.get(1).getPose().getTranslation(), .25),
                traj.getInterpolated(0.25).state().getPose().getTranslation());
        assertEquals(0, traj.getInterpolated(0.25).index_floor());
        assertEquals(1, traj.getInterpolated(0.25).index_ceil());
        assertEquals(kWaypoints.get(1).getPose().getTranslation().interpolate(kWaypoints.get(2).getPose().getTranslation(), .5),
                traj.getInterpolated(1.5).state().getPose().getTranslation());
        assertEquals(1, traj.getInterpolated(1.5).index_floor());
        assertEquals(2, traj.getInterpolated(1.5).index_ceil());
        assertEquals(kWaypoints.get(2).getPose().getTranslation().interpolate(kWaypoints.get(3).getPose().getTranslation(), .75),
                traj.getInterpolated(2.75).state().getPose().getTranslation());
        assertEquals(2, traj.getInterpolated(2.75).index_floor());
        assertEquals(3, traj.getInterpolated(2.75).index_ceil());

        assertEquals(kHeadings.get(0).get().interpolate(kHeadings.get(1).get(), .25),
                traj.getInterpolated(0.25).heading().get());
        assertEquals(0, traj.getInterpolated(0.25).index_floor());
        assertEquals(1, traj.getInterpolated(0.25).index_ceil());
        assertEquals(kHeadings.get(1).get().interpolate(kHeadings.get(2).get(), .5),
                traj.getInterpolated(1.5).heading().get());
        assertEquals(1, traj.getInterpolated(1.5).index_floor());
        assertEquals(2, traj.getInterpolated(1.5).index_ceil());
        assertEquals(kHeadings.get(2).get().interpolate(kHeadings.get(3).get(), .75),
                traj.getInterpolated(2.75).heading().get());
        assertEquals(2, traj.getInterpolated(2.75).index_floor());
        assertEquals(3, traj.getInterpolated(2.75).index_ceil());

        // Trajectory<Translation2dState, Rotation2dState>.IndexView index_view =
        // traj.getIndexView();
        PathSamplePoint sample0 = traj.getInterpolated(0.25);
        // index_view.sample(0.25);
        assertEquals(kWaypoints.get(0).getPose().getTranslation().interpolate(kWaypoints.get(1).getPose().getTranslation(), .25),
                sample0.state().getPose().getTranslation());

        PathSamplePoint sample1 = traj.getInterpolated(1.5);
        // index_view.sample(1.5);
        assertEquals(kWaypoints.get(1).getPose().getTranslation().interpolate(kWaypoints.get(2).getPose().getTranslation(), .5),
                sample1.state().getPose().getTranslation());

        PathSamplePoint sample2 = traj.getInterpolated(2.75);
        // index_view.sample(2.75);
        assertEquals(kWaypoints.get(2).getPose().getTranslation().interpolate(kWaypoints.get(3).getPose().getTranslation(), .75),
                sample2.state().getPose().getTranslation());

        PathSamplePoint sample3 = traj.getInterpolated(0.25);
        // index_view.sample(0.25);
        assertEquals(kHeadings.get(0).get().interpolate(kHeadings.get(1).get(), .25),
                sample3.heading().get());

        PathSamplePoint sample4 = traj.getInterpolated(1.5);
        // index_view.sample(1.5);
        assertEquals(kHeadings.get(1).get().interpolate(kHeadings.get(2).get(), .5),
                sample4.heading().get());

        PathSamplePoint sample5 = traj.getInterpolated(2.75);
        // index_view.sample(2.75);
        assertEquals(kHeadings.get(2).get().interpolate(kHeadings.get(3).get(), .75),
                sample5.heading().get());
    }
}
