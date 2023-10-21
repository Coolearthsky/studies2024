package frc;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.arm.ArmTrajectories;
import frc.robot.armMotion.ArmAngles;

public class test {

    @Test
    void testUnreachable() {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories trajectories = new ArmTrajectories(config);
        ArmAngles t0 = new ArmAngles(0.089803, 1.681915);
        Trajectory trajectory = trajectories.makeTrajectory(t0);
        assertNotNull(trajectory);
    }

}
