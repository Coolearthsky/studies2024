package com.team254.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import com.team254.lib.geometry.Pose2dWithCurvature;

import edu.wpi.first.math.geometry.Translation2d;

public class TimedStateTest {
    public static final double kTestEpsilon = 1e-12;

    @Test
    public void test() {
        // At (0,0,0), t=0, v=0, acceleration=1
        final TimedPose start_state = new TimedPose(
                new Pose2dWithCurvature(
                        GeometryUtil.fromTranslation(new Translation2d(0.0, 0.0)), 0.0),
                0.0, 0.0, 1.0);

        // At (.5,0,0), t=1, v=1, acceleration=0
        final TimedPose end_state = new TimedPose(
                new Pose2dWithCurvature(
                        GeometryUtil.fromTranslation(new Translation2d(0.5, 0.0)), 0.0),
                1.0, 1.0, 0.0);

        TimedPose i0 = start_state.interpolate2(end_state, 0.0);
        assertEquals(start_state, i0, String.format("%s %s", start_state, i0));
        assertEquals(end_state, start_state.interpolate2(end_state, 1.0));
        assertEquals(end_state, end_state.interpolate2(start_state, 0.0));
        System.out.println(end_state.interpolate2(start_state, 1.0));
        assertEquals(start_state, end_state.interpolate2(start_state, 1.0));

        final TimedPose intermediate_state = start_state.interpolate2(end_state, 0.5);
        assertEquals(0.5, intermediate_state.t(), kTestEpsilon);
        assertEquals(start_state.acceleration(), intermediate_state.acceleration(), kTestEpsilon);
        assertEquals(0.5, intermediate_state.velocity(), kTestEpsilon);
        assertEquals(0.125, intermediate_state.state().getPose().getTranslation().getX(), kTestEpsilon);
    }
}
