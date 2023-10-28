package com.team254.frc2022.planners;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.swerve.ChassisSpeeds;

import com.team254.frc2022.Constants;
import com.team254.lib.control.Lookahead;
import com.team254.lib.geometry.Pose2dState;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.geometry.Twist2dWrapper;
import com.team254.lib.physics.SwerveDrive;
import com.team254.lib.trajectory.DistanceView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.TrajectorySamplePoint;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.SwerveDriveDynamicsConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionPlanner {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(1.0);

    public enum FollowerType {
        FEEDFORWARD_ONLY,
        PID,
        PURE_PURSUIT
    }

    FollowerType mFollowerType = FollowerType.PID;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    final SwerveDrive mModel;

    private double defaultCook = 0.4;
    private boolean useDefaultCook = true;

    private TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2dState>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mLastPathSetpoint = null;
    public TimedState<Pose2dWithCurvature> mPathSetpoint = new TimedState<>(GeometryUtil.kPose2dWithCurvatureIdentity);
    public TimedState<Rotation2dState> mHeadingSetpoint = null;
    public TimedState<Rotation2dState> mLastHeadingSetpoint = new TimedState<>(GeometryUtil.kRotationIdentity);

    public double mVelocitym = 0;
    Pose2dState mError = GeometryUtil.kPose2dIdentity;

    Translation2dState mTranslationalError = GeometryUtil.kTranslation2dIdentity;
    Rotation2dState mHeadingError = GeometryUtil.kRotationIdentity;
    Rotation2dState mInitialHeading = GeometryUtil.kRotationIdentity;
    Rotation2dState mRotationDiff = GeometryUtil.kRotationIdentity;
    Pose2dState mCurrentState = GeometryUtil.kPose2dIdentity;

    double mCurrentTrajectoryLength = 0.0;
    double mTotalTime = Double.POSITIVE_INFINITY;
    double mStartTime = Double.POSITIVE_INFINITY;
    double mDTheta = 0.0;
    ChassisSpeeds mOutput = new ChassisSpeeds();

    Lookahead mSpeedLookahead = null;

    double mDt = 0.0;

    public DriveMotionPlanner() {
        mModel = new SwerveDrive(
                0.0942 / 2,
                0.464 / 2.0 * Constants.kTrackScrubFactor);

        SmartDashboard.putString("Steering Direction", "");
        SmartDashboard.putString("Last Pose", "");
        SmartDashboard.putString("Current Pose", "");
        SmartDashboard.putNumber("Finished Traj?", -1.0);
        SmartDashboard.putNumber("Adaptive Lookahead", -1.0);
    }

    public void setTrajectory(
            final TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2dState>> trajectory) {

        mCurrentTrajectory = trajectory;

        if (mCurrentTrajectory == null) {
            System.out.println("YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
        }
        mPathSetpoint = trajectory.getState();
        mHeadingSetpoint = trajectory.getHeading();
        mLastHeadingSetpoint = null;
        mLastPathSetpoint = null;
        useDefaultCook = true;
        mSpeedLookahead = new Lookahead(Constants.kAdaptivePathMinLookaheadDistance,
                Constants.kAdaptivePathMaxLookaheadDistance, 0.0,
                Units.meters_to_inches(Constants.kMaxVelocityMetersPerSecond));
        mCurrentTrajectoryLength = mCurrentTrajectory.trajectory().getLastPoint().state().t();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getPoint(i).state().velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getPoint(i).state().velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mTranslationalError = GeometryUtil.kTranslation2dIdentity;
        mHeadingError = GeometryUtil.kRotationIdentity;
        mLastHeadingSetpoint = null;
        mLastPathSetpoint = null;
        mOutput = new ChassisSpeeds();
        mLastTime = Double.POSITIVE_INFINITY;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2dState>> generateTrajectory(
            boolean reversed,
            final List<Pose2dState> waypoints,
            final List<Rotation2dState> headings,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {
        return generateTrajectory(reversed, waypoints, headings, constraints, 0.0, 0.0, max_vel, max_accel,
                max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2dState>> generateTrajectory(
            boolean reversed,
            final List<Pose2dState> waypoints,
            final List<Rotation2dState> headings,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,
            double end_vel,
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {
        List<Pose2dState> waypoints_maybe_flipped = waypoints;
        List<Rotation2dState> headings_maybe_flipped = headings;
        final Pose2dState flip = GeometryUtil.fromRotation(new Rotation2dState(-1, 0));
        // TODO re-architect the spline generator to support reverse.
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            headings_maybe_flipped = new ArrayList<>(headings.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped
                        .add(new Pose2dState(GeometryUtil.transformBy(waypoints.get(i).get(), flip.get())));
                headings_maybe_flipped.add(headings.get(i).rotateBy(flip.get().getRotation()));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature, Rotation2dState> trajectory = TrajectoryUtil.trajectoryFromWaypoints(
                waypoints_maybe_flipped, headings_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithCurvature> flipped_points = new ArrayList<>(trajectory.length());
            List<Rotation2dState> flipped_headings = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped_points
                        .add(new Pose2dWithCurvature(
                                GeometryUtil.transformBy(trajectory.getPoint(i).state().getPose().get(), flip.pose2d),
                                -trajectory
                                        .getPoint(i).state().getCurvature(),
                                trajectory.getPoint(i).state().getDCurvatureDs()));
                flipped_headings.add(trajectory.getPoint(i).heading().rotateBy(flip.get().getRotation()));
            }
            trajectory = new Trajectory<>(flipped_points, flipped_headings);
        }

        // Create the constraint that the robot must be able to traverse the trajectory
        // without ever applying more
        // than the specified voltage.
        final SwerveDriveDynamicsConstraint drive_constraints = new SwerveDriveDynamicsConstraint(mModel, max_voltage);
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }

        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2dState>> timed_trajectory = TimingUtil
                .timeParameterizeTrajectory(reversed, new DistanceView<>(trajectory), kMaxDx, Arrays.asList(),
                        start_vel, end_vel, max_vel, max_accel);
        return timed_trajectory;
    }

    protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
        // Feedback on longitudinal error (distance).
        final double kPathk = 0.6/*
                                  * * Math.hypot(chassisSpeeds.vxMetersPerSecond,
                                  * chassisSpeeds.vyMetersPerSecond)
                                  */;// 0.15;
        final double kPathKTheta = 0.3;
        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + kPathk * Units.inches_to_meters(
                mError.get().getTranslation().getX());
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + kPathk * Units.inches_to_meters(
                mError.get().getTranslation().getY());
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
                + kPathKTheta * mError.get().getRotation().getRadians();
        return chassisSpeeds;
    }

    protected ChassisSpeeds updatePurePursuit(Pose2dState current_state, double feedforwardOmegaRadiansPerSecond) {
        double lookahead_time = Constants.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mPathSetpoint.state().distance(lookahead_state.state());
        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mPathSetpoint.velocity())
                + Constants.kAdaptiveErrorLookaheadCoefficient * mError.get().getTranslation().getNorm();
        SmartDashboard.putNumber("Adaptive Lookahead", adaptive_lookahead_distance);
        // Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mPathSetpoint.state().distance(lookahead_state.state());
        }

        // If the Lookahead Point's Distance is less than the Lookahead Distance
        // transform it so it is the lookahead distance away
        if (actual_lookahead_distance < adaptive_lookahead_distance) {
            lookahead_state = new TimedState<>(
                    new Pose2dWithCurvature(
                            GeometryUtil.transformBy(lookahead_state.state().getPose().get(),
                                    GeometryUtil.fromTranslation(new Translation2dState(
                                            (mIsReversed ? -1.0 : 1.0) * (Constants.kPathMinLookaheadDistance -
                                                    actual_lookahead_distance),
                                            0.0)).get()),
                            0.0),
                    lookahead_state.t(), lookahead_state.velocity(), lookahead_state.acceleration());
        }

        SmartDashboard.putNumber("Path X", lookahead_state.state().getTranslation().getX());
        SmartDashboard.putNumber("Path Y", lookahead_state.state().getTranslation().getY());
        SmartDashboard.putNumber("Path Velocity", lookahead_state.velocity());

        // Find the vector between robot's current position and the lookahead state
        Translation2dState lookaheadTranslation = new Translation2dState(
                current_state.get().getTranslation(),
                lookahead_state.state().getTranslation());

        // Set the steering direction as the direction of the vector
        Rotation2dState steeringDirection = new Rotation2dState(lookaheadTranslation.get().getAngle());
        SmartDashboard.putString("Steering Direction", steeringDirection.toString());

        // Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(GeometryUtil.inverse(current_state).getRotation());
        SmartDashboard.putString("Steering Direction After Rotation", steeringDirection.toString());

        // Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeed = Math.abs(mPathSetpoint.velocity())
                / Units.meters_to_inches(Constants.kMaxVelocityMetersPerSecond);

        // The Default Cook is the minimum speed to use. So if a feedforward speed is
        // less than defaultCook, the robot will drive at the defaultCook speed
        if (normalizedSpeed > defaultCook || mPathSetpoint.t() > (mCurrentTrajectoryLength / 2.0)) {
            useDefaultCook = false;
        }
        if (useDefaultCook) {
            normalizedSpeed = defaultCook;
        }

        // Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate
        // (Vx, Vy) in Robot Frame
        final Translation2dState steeringVector = new Translation2dState(
                steeringDirection.get().getCos() * normalizedSpeed,
                steeringDirection.get().getSin() * normalizedSpeed);
        SmartDashboard.putString("Steering Vector", steeringVector.toString());
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                steeringVector.get().getX() * Constants.kMaxVelocityMetersPerSecond,
                steeringVector.get().getY() * Constants.kMaxVelocityMetersPerSecond, feedforwardOmegaRadiansPerSecond);

        // Use the P-Controller for To Follow the Time-Parametrized Heading
        final double kPathKTheta = 0.3;

        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
                + kPathKTheta * mError.get().getRotation().getRadians();
        return chassisSpeeds;
    }

    public ChassisSpeeds update(double timestamp, Pose2dState current_state) {

        if (mCurrentTrajectory == null) {
            // System.out.println("YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
            return null;
        }

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;

            mInitialHeading = mCurrentTrajectory.trajectory().getPoint(0).heading().state();
            var finalHeading = mCurrentTrajectory.trajectory().getLastPoint().heading().state();
            mTotalTime = mCurrentTrajectory.trajectory().getLastPoint().state().t() -
                    mCurrentTrajectory.trajectory().getPoint(0).state().t();
            // Interpolate heading
            mRotationDiff = finalHeading.rotateBy(mInitialHeading.unaryMinus());
            if (mRotationDiff.get().getRadians() > Math.PI) {
                mRotationDiff = mRotationDiff.unaryMinus().rotateBy(GeometryUtil.fromRadians(Math.PI));
            }

            mStartTime = timestamp;
            if (Math.abs(mRotationDiff.get().getRadians()) < 0.1) {
                mDTheta = 0.0;
            } else {
                mDTheta = mRotationDiff.get().getRadians() / mTotalTime;
            }
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>, TimedState<Rotation2dState>> sample_point;

        mHeadingSetpoint = new TimedState<>(mInitialHeading.rotateBy(mRotationDiff.times(Math.min(1.0,
                (timestamp - mStartTime) / mTotalTime))));
        mCurrentState = current_state;
        if (!isDone()) {
            sample_point = mCurrentTrajectory.advance(mDt);
            // Compute error in robot frame
            mError = new Pose2dState(GeometryUtil.transformBy(GeometryUtil.inverse(current_state),
                    mPathSetpoint.state().getPose().get()));
            mError = new Pose2dState(mError.get().getTranslation(),
                    current_state.get().getRotation().unaryMinus()
                            .rotateBy(mHeadingSetpoint.state().getRotation().get()));

            if (mFollowerType == FollowerType.PID) {

                mPathSetpoint = sample_point.state();
                // System.out.println(mPathSetpoint);

                // System.out.println(mPathSetpoint.velocity());

                // Generate feedforward voltages.
                // final double velocity_m = Units.inches_to_meters(mPathSetpoint.velocity());

                final double velocity_m = mPathSetpoint.velocity();
                mVelocitym = velocity_m;
                // final double velocity_m = mPathSetpoint.velocity();

                // System.out.println("VELLLLLLLLLLLLLCOOOOOCIIITYYYYY " + velocity_m);
                final Rotation2dState rotation = mPathSetpoint.state().getRotation();

                // In field frame
                var chassis_v = new Translation2dState(rotation.get().getCos() * velocity_m,
                        rotation.get().getSin() * velocity_m);
                // Convert to robot frame
                chassis_v = new Translation2dState(
                        chassis_v.get().rotateBy(mHeadingSetpoint.state().getRotation().unaryMinus().get()));

                var chassis_twist = new Twist2dWrapper(
                        chassis_v.get().getX(),
                        chassis_v.get().getY(), mDTheta);

                // System.out.println(chassis_twist);

                var chassis_speeds = new ChassisSpeeds(
                        chassis_twist.dx, chassis_twist.dy, chassis_twist.dtheta);
                // PID is in robot frame

                mOutput = updatePIDChassis(chassis_speeds);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                double searchStepSize = 1.0;
                double previewQuantity = 0.0;
                double searchDirection = 1.0;
                double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
                double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
                searchDirection = Math.signum(reverseDistance - forwardDistance);
                while (searchStepSize > 0.001) {
                    if (Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.01))
                        break;
                    while (/* next point is closer than current point */ distance(current_state, previewQuantity
                            + searchStepSize * searchDirection) < distance(current_state, previewQuantity)) {
                        /* move to next point */
                        previewQuantity += searchStepSize * searchDirection;
                    }
                    searchStepSize /= 10.0;
                    searchDirection *= -1;
                }
                sample_point = mCurrentTrajectory.advance(previewQuantity);
                mPathSetpoint = sample_point.state();
                SmartDashboard.putString("Current Pose", mCurrentState.get().toString());
                SmartDashboard.putString("Last Pose",
                        mCurrentTrajectory.trajectory().getLastPoint().state().state().getPose().toString());
                SmartDashboard.putNumber("Finished Traj?", mPathSetpoint.velocity());

                mOutput = updatePurePursuit(current_state, mDTheta);

            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new ChassisSpeeds();
        }
        return mOutput;
    }

    public ChassisSpeeds update(
            TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2dState>> trajectory,
            double timestamp, Pose2dState current_state) {
        if (trajectory == null) {
            // System.out.println("YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
            return null;
        }

        if (trajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;

            mInitialHeading = new Rotation2dState(
                    trajectory.trajectory().getPoint(0).heading().state().get());
            var finalHeading = trajectory.trajectory().getLastPoint().heading().state();
            mTotalTime = trajectory.trajectory().getLastPoint().state().t() -
                    trajectory.trajectory().getPoint(0).state().t();
            // Interpolate heading
            mRotationDiff = finalHeading.rotateBy(mInitialHeading.unaryMinus());
            if (mRotationDiff.get().getRadians() > Math.PI) {
                mRotationDiff = mRotationDiff.unaryMinus().rotateBy(GeometryUtil.fromRadians(Math.PI));
            }

            mStartTime = timestamp;
            if (Math.abs(mRotationDiff.get().getRadians()) < 0.1) {
                mDTheta = 0.0;
            } else {
                mDTheta = mRotationDiff.get().getRadians() / mTotalTime;
            }
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>, TimedState<Rotation2dState>> sample_point;

        mHeadingSetpoint = new TimedState<>(mInitialHeading.rotateBy(mRotationDiff.times(Math.min(1.0,
                (timestamp - mStartTime) / mTotalTime))));
        mCurrentState = current_state;
        if (!isDone()) {
            sample_point = trajectory.advance(mDt);
            // Compute error in robot frame
            mError = new Pose2dState(GeometryUtil.transformBy(GeometryUtil.inverse(current_state),
                    mPathSetpoint.state().getPose().get()));
            mError = new Pose2dState(mError.get().getTranslation(),
                    current_state.get().getRotation().unaryMinus()
                            .rotateBy(mHeadingSetpoint.state().getRotation().get()));

            if (mFollowerType == FollowerType.PID) {
                mPathSetpoint = sample_point.state();

                // Generate feedforward voltages.
                final double velocity_m = Units.inches_to_meters(mPathSetpoint.velocity());
                final Rotation2dState rotation = mPathSetpoint.state().getRotation();

                // In field frame
                var chassis_v = new Translation2dState(rotation.get().getCos() * velocity_m,
                        rotation.get().getSin() * velocity_m);
                // Convert to robot frame
                chassis_v = new Translation2dState(
                        chassis_v.get().rotateBy(mHeadingSetpoint.state().getRotation().unaryMinus().get()));

                var chassis_twist = new Twist2dWrapper(
                        chassis_v.get().getX(),
                        chassis_v.get().getY(), mDTheta);

                var chassis_speeds = new ChassisSpeeds(
                        chassis_twist.dx, chassis_twist.dy, chassis_twist.dtheta);
                // PID is in robot frame
                mOutput = updatePIDChassis(chassis_speeds);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                double searchStepSize = 1.0;
                double previewQuantity = 0.0;
                double searchDirection = 1.0;
                double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
                double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
                searchDirection = Math.signum(reverseDistance - forwardDistance);
                while (searchStepSize > 0.001) {
                    if (Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.01))
                        break;
                    while (/* next point is closer than current point */ distance(current_state, previewQuantity
                            + searchStepSize * searchDirection) < distance(current_state, previewQuantity)) {
                        /* move to next point */
                        previewQuantity += searchStepSize * searchDirection;
                    }
                    searchStepSize /= 10.0;
                    searchDirection *= -1;
                }
                sample_point = trajectory.advance(previewQuantity);
                mPathSetpoint = sample_point.state();
                SmartDashboard.putString("Current Pose", mCurrentState.get().toString());
                SmartDashboard.putString("Last Pose",
                        trajectory.trajectory().getLastPoint().state().state().getPose().toString());
                SmartDashboard.putNumber("Finished Traj?", mPathSetpoint.velocity());

                mOutput = updatePurePursuit(current_state, mDTheta);

            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new ChassisSpeeds();
        }
        return mOutput;
    }

    public double getVelocitySetpoint() {
        return mVelocitym;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && (mCurrentTrajectory.isDone());
    }

    public synchronized Translation2dState getTranslationalError() {
        return new Translation2dState(
                Units.inches_to_meters(mError.get().getTranslation().getX()),
                Units.inches_to_meters(mError.get().getTranslation().getY()));
    }

    public synchronized Rotation2dState getHeadingError() {
        return new Rotation2dState(mError.get().getRotation());
    }

    private double distance(Pose2dState current_state, double additional_progress) {
        return mCurrentTrajectory.preview(additional_progress).state().state().getPose().distance(current_state);
    }

    public synchronized TimedState<Pose2dWithCurvature> getPathSetpoint() {
        return mPathSetpoint;
    }

    public synchronized TimedState<Rotation2dState> getHeadingSetpoint() {
        return mHeadingSetpoint;
    }
}