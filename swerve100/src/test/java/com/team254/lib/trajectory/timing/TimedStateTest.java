package com.team254.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import com.team254.lib.geometry.Pose2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.util.Util;

public class TimedStateTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // At (0,0,0), t=0, v=0, acceleration=1
        final TimedState<Pose2dState> start_state = new TimedState<>(Pose2dState.fromTranslation(new Translation2dState(0.0, 0.0)),
                0.0, 0.0, 1.0);

        // At (.5,0,0), t=1, v=1, acceleration=0
        final TimedState<Pose2dState> end_state = new TimedState<>(Pose2dState.fromTranslation(new Translation2dState(0.5, 0.0)), 1.0,
                1.0, 0.0);

        TimedState<Pose2dState> i0 = start_state.interpolate2(end_state, 0.0);
        assertEquals(start_state, i0, String.format("%s %s", start_state, i0));
        assertEquals(end_state, start_state.interpolate2(end_state, 1.0));
        assertEquals(end_state, end_state.interpolate2(start_state, 0.0));
        System.out.println(end_state.interpolate2(start_state, 1.0));
        assertEquals(start_state, end_state.interpolate2(start_state, 1.0));

        final TimedState<Pose2dState> intermediate_state = start_state.interpolate2(end_state, 0.5);
        assertEquals(0.5, intermediate_state.t(), kTestEpsilon);
        assertEquals(start_state.acceleration(), intermediate_state.acceleration(), kTestEpsilon);
        assertEquals(0.5, intermediate_state.velocity(), kTestEpsilon);
        assertEquals(0.125, intermediate_state.state().get().getTranslation().getX(), kTestEpsilon);
    }

}
