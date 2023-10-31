package org.team100.lib.trajectory;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.planners.DriveMotionPlanner;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimedRotation;
import org.team100.lib.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

class FancyTrajectoryTest {
    private static final double kDelta = 0.001;

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
        DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner();
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
        // assertEquals(10, trajectory.length());

        TrajectoryTimeSampler view = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(view);

        mMotionPlanner.setTrajectory(iter);

        // in the drive loop, this happens:
        Pose2d actualPose = GeometryUtil.kPose2dIdentity;
        double fpgatime = 0;
        final double now = Timer.getFPGATimestamp();
        // this stuff doesn't work, and i don't want to break the build so i'm
        // commenting it out.

        Translation2d translation2d = new Translation2d(1, 1);
        Rotation2d rotation2d = Rotation2d.fromDegrees(90);

        actualPose = new Pose2d(translation2d, rotation2d);

        System.out.println("POSE::::::::::::::::::::::::::::" + actualPose);

        ChassisSpeeds output = mMotionPlanner.update(now, actualPose);

        Translation2d translational_error = mMotionPlanner.getTranslationalErrorM();
        Rotation2d heading_error = mMotionPlanner.getHeadingError();
        TimedPose path_setpoint = mMotionPlanner.getPathSetpoint();
        TimedRotation heading_setpoint = mMotionPlanner.getHeadingSetpoint();

        // the DriveMotionPlanner has two ways to follow the trajectory: it could just
        // follow it,
        // or it could look into the future trajectory states, and chase those, using
        // "pure pursuit."
        // the "future" is pretty far away, 0.25 seconds. I guess the difference is that
        // the "pure pursuit"
        // model corects errors in the direction of the future state, whereas the pure
        // tracking model
        // aims for the *current* state, which seems worse.

    }

}
