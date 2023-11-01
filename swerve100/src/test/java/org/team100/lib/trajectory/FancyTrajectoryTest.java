package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.planners.DriveMotionPlanner;
import org.team100.lib.planners.DriveMotionPlanner.FollowerType;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimedRotation;
import org.team100.lib.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class FancyTrajectoryTest {

    /**
     * This is derived from one of the auton trajectories in
     * TrajectoryGenerator.getFarRightStartToFarRightBallHalf
     * it moves to the right in a straight line.
     */
    @Test
    void testLikeAuton() {
        final double kMaxVel = 1.0;
        final double kMaxAccel = 1.0;
        // this doesn't actually do anything.
        final double kMaxVoltage = 9.0;

        // first right and then ahead
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(10, -10, Rotation2d.fromDegrees(0)));
        // while turning 180
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));
        // these don't actually do anything.
        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner(FollowerType.PID);
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory trajectory = mMotionPlanner.generateTrajectory(
                waypoints,
                headings,
                constraints,
                start_vel,
                end_vel,
                kMaxVel,
                kMaxAccel,
                kMaxVoltage);
        System.out.println(trajectory);
        System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        assertEquals(10, trajectory.length());

        TrajectoryTimeSampler view = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(view);

        mMotionPlanner.setTrajectory(iter);

        // this is a series of perfect trajectory following states,
        // based on the trajectory itself.

        {
            ChassisSpeeds output = mMotionPlanner.update(0, 
            new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(1.57079632679)));
            assertEquals(0, output.vxMetersPerSecond, 0.001);
            assertEquals(0, output.vyMetersPerSecond, 0.001);
            assertEquals(0.083, output.omegaRadiansPerSecond, 0.001);
        }

        {
            ChassisSpeeds output = mMotionPlanner.update(4.0,
                    new Pose2d(new Translation2d(0.05, -2.0), Rotation2d.fromRadians(-1.5)));
            assertEquals(0, output.vxMetersPerSecond, 0.001);
            assertEquals(0, output.vyMetersPerSecond, 0.001);
            assertEquals(0, output.omegaRadiansPerSecond, 0.001);
            Translation2d translational_error = mMotionPlanner.getTranslationalErrorM();
            assertEquals(0, translational_error.getX(), 0.001);
            assertEquals(0, translational_error.getY(), 0.001);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.001);
            TimedPose path_setpoint = mMotionPlanner.getPathSetpoint();
            assertEquals(0, path_setpoint.state().poseMeters.getX(), 0.001);
            assertEquals(0, path_setpoint.state().poseMeters.getY(), 0.001);
            assertEquals(0, path_setpoint.state().poseMeters.getRotation().getRadians(), 0.001);
            assertEquals(0, path_setpoint.t(), 0.001);
            assertEquals(0, path_setpoint.velocity(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            TimedRotation heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            assertEquals(0, heading_setpoint.state().getRadians(), 0.001);
            assertEquals(0, heading_setpoint.t(), 0.001);
            assertEquals(0, heading_setpoint.velocity(), 0.001);
            assertEquals(0, heading_setpoint.acceleration(), 0.001);
        }
        {
            ChassisSpeeds output = mMotionPlanner.update(6.0,
                    new Pose2d(new Translation2d(0.34, -3.98), Rotation2d.fromRadians(-1.33)));
            assertEquals(0, output.vxMetersPerSecond, 0.001);
            assertEquals(0, output.vyMetersPerSecond, 0.001);
            assertEquals(0, output.omegaRadiansPerSecond, 0.001);
            Translation2d translational_error = mMotionPlanner.getTranslationalErrorM();
            assertEquals(0, translational_error.getX(), 0.001);
            assertEquals(0, translational_error.getY(), 0.001);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.001);
            TimedPose path_setpoint = mMotionPlanner.getPathSetpoint();
            assertEquals(0, path_setpoint.state().poseMeters.getX(), 0.001);
            assertEquals(0, path_setpoint.state().poseMeters.getY(), 0.001);
            assertEquals(0, path_setpoint.state().poseMeters.getRotation().getRadians(), 0.001);
            assertEquals(0, path_setpoint.t(), 0.001);
            assertEquals(0, path_setpoint.velocity(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            TimedRotation heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            assertEquals(0, heading_setpoint.state().getRadians(), 0.001);
            assertEquals(0, heading_setpoint.t(), 0.001);
            assertEquals(0, heading_setpoint.velocity(), 0.001);
            assertEquals(0, heading_setpoint.acceleration(), 0.001);
        }
    }
}
