package org.team100.lib.planners;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.team100.lib.controller.Lookahead;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.path.Path;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.physics.SwerveDrive;
import org.team100.lib.spline.Spline;
import org.team100.lib.spline.SplineGenerator;
import org.team100.lib.timing.SwerveDriveDynamicsConstraint;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimedRotation;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.trajectory.Trajectory;
import org.team100.lib.trajectory.TrajectorySamplePoint;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

import org.team100.lib.spline.QuinticHermiteSpline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionPlanner {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(1.0);

    // Pure Pursuit Constants
    public static final double kPathLookaheadTime = 0.25; // From 1323 (2019)
    public static final double kPathMinLookaheadDistance = 12.0; // From 1323 (2019)
    public static final double kAdaptivePathMinLookaheadDistance = 6.0;
    public static final double kAdaptivePathMaxLookaheadDistance = 24.0;
    public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

    public static final double kTrackScrubFactor = 1;

    public static final double kMaxVelocityMetersPerSecond = 4.959668;

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

    private TrajectoryTimeIterator mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedPose mLastPathSetpoint = null;
    public TimedPose mPathSetpoint = new TimedPose(new PoseWithCurvature());
    public TimedRotation mHeadingSetpoint = null;
    public TimedRotation mLastHeadingSetpoint = new TimedRotation(new Rotation2d());

    public double mVelocitym = 0;
    Pose2d mError = GeometryUtil.kPose2dIdentity;

    // TODO: this appears not do be used?
    Translation2d mTranslationalError = GeometryUtil.kTranslation2dIdentity;
    // TODO: this appears not do be used?
    Rotation2d mHeadingError = GeometryUtil.kRotationIdentity;
    Rotation2d mInitialHeading = GeometryUtil.kRotationIdentity;
    Rotation2d mRotationDiff = GeometryUtil.kRotationIdentity;
    Pose2d mCurrentState = GeometryUtil.kPose2dIdentity;

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
                0.464 / 2.0 * kTrackScrubFactor);
    }

    public void setTrajectory(final TrajectoryTimeIterator trajectory) {

        mCurrentTrajectory = trajectory;

        if (mCurrentTrajectory == null) {
            System.out.println("YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
        }
        mPathSetpoint = trajectory.getState();
        mHeadingSetpoint = trajectory.getHeading();
        mLastHeadingSetpoint = null;
        mLastPathSetpoint = null;
        useDefaultCook = true;
        mSpeedLookahead = new Lookahead(kAdaptivePathMinLookaheadDistance,
                kAdaptivePathMaxLookaheadDistance, 0.0,
                Units.metersToInches(kMaxVelocityMetersPerSecond));
        mCurrentTrajectoryLength = mCurrentTrajectory.trajectory().getLastPoint().state().t();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getPoint(i).state().velocity() > 1e-12) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getPoint(i).state().velocity() < -1e-12) {
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

    public Trajectory generateTrajectory(
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_voltage) {

        // Create a path from splines.

        List<QuinticHermiteSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i1 = 1; i1 < waypoints.size(); ++i1) {
            splines.add(new QuinticHermiteSpline(waypoints.get(i1 - 1), waypoints.get(i1)));
        }
        QuinticHermiteSpline.optimizeSpline(splines);
        final List<? extends Spline> splines1 = splines;

        Path trajectory = new Path(SplineGenerator.parameterizeSplines(splines1, headings, kMaxDx, kMaxDy, kMaxDTheta));

        // Create the constraint that the robot must be able to traverse the trajectory
        // without ever applying more
        // than the specified voltage.
        final SwerveDriveDynamicsConstraint drive_constraints = new SwerveDriveDynamicsConstraint(mModel, max_voltage);
        List<TimingConstraint> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }

        // Generate the timed trajectory.
        PathDistanceSampler distance_view = new PathDistanceSampler(trajectory);
        return TimingUtil.timeParameterizeTrajectory(distance_view, kMaxDx, Arrays.asList(),
                start_vel, end_vel, max_vel, max_accel);
    }

    protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
        // Feedback on longitudinal error (distance).
        final double kPathk = 0.6/*
                                  * * Math.hypot(chassisSpeeds.vxMetersPerSecond,
                                  * chassisSpeeds.vyMetersPerSecond)
                                  */;// 0.15;
        final double kPathKTheta = 0.3;
        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + kPathk * Units.inchesToMeters(
                mError.getTranslation().getX());
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + kPathk * Units.inchesToMeters(
                mError.getTranslation().getY());
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
                + kPathKTheta * mError.getRotation().getRadians();
        return chassisSpeeds;
    }

    protected ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond) {
        double lookahead_time = kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedPose lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();

        PoseWithCurvature lookstate = lookahead_state.state();
        PoseWithCurvature setpointstate = mPathSetpoint.state();
        final PoseWithCurvature other1 = lookstate;
        double actual_lookahead_distance = GeometryUtil.norm(
                GeometryUtil.slog(
                        GeometryUtil.transformBy(
                                GeometryUtil.inverse(setpointstate.poseMeters),
                                other1.poseMeters)));

        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mPathSetpoint.velocity())
                + kAdaptiveErrorLookaheadCoefficient * mError.getTranslation().getNorm();
        SmartDashboard.putNumber("Adaptive Lookahead", adaptive_lookahead_distance);
        // Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();

            PoseWithCurvature state = lookahead_state.state();
            PoseWithCurvature state2 = mPathSetpoint.state();
            final PoseWithCurvature other = state;
            actual_lookahead_distance = GeometryUtil.norm(
                    GeometryUtil.slog(
                            GeometryUtil.transformBy(
                                    GeometryUtil.inverse(state2.poseMeters),
                                    other.poseMeters)));
        }

        // If the Lookahead Point's Distance is less than the Lookahead Distance
        // transform it so it is the lookahead distance away
        if (actual_lookahead_distance < adaptive_lookahead_distance) {
            lookahead_state = new TimedPose(
                    new PoseWithCurvature(
                            GeometryUtil.transformBy(lookahead_state.state().poseMeters,
                                    GeometryUtil.fromTranslation(
                                            new Translation2d(
                                                    (mIsReversed ? -1.0 : 1.0) * (kPathMinLookaheadDistance -
                                                            actual_lookahead_distance),
                                                    0.0))),
                            0.0),
                    lookahead_state.t(), lookahead_state.velocity(), lookahead_state.acceleration());
        }

        SmartDashboard.putNumber("Path X", lookahead_state.state().poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("Path Y", lookahead_state.state().poseMeters.getTranslation().getY());
        SmartDashboard.putNumber("Path Velocity", lookahead_state.velocity());

        // Find the vector between robot's current position and the lookahead state
        Translation2d lookaheadTranslation = lookahead_state.state().poseMeters.getTranslation()
                .minus(current_state.getTranslation());

        // Set the steering direction as the direction of the vector
        Rotation2d steeringDirection = lookaheadTranslation.getAngle();
        SmartDashboard.putString("Steering Direction", steeringDirection.toString());

        // Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(GeometryUtil.inverse(current_state).getRotation());
        SmartDashboard.putString("Steering Direction After Rotation", steeringDirection.toString());

        // Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeed = Math.abs(mPathSetpoint.velocity())
                / Units.metersToInches(kMaxVelocityMetersPerSecond);

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
        final Translation2d steeringVector = new Translation2d(
                steeringDirection.getCos() * normalizedSpeed,
                steeringDirection.getSin() * normalizedSpeed);
        SmartDashboard.putString("Steering Vector", steeringVector.toString());
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                steeringVector.getX() * kMaxVelocityMetersPerSecond,
                steeringVector.getY() * kMaxVelocityMetersPerSecond, feedforwardOmegaRadiansPerSecond);

        // Use the P-Controller for To Follow the Time-Parametrized Heading
        final double kPathKTheta = 0.3;

        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
                + kPathKTheta * mError.getRotation().getRadians();
        return chassisSpeeds;
    }

    public ChassisSpeeds update(double timestamp, Pose2d current_state) {

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
            if (mRotationDiff.getRadians() > Math.PI) {
                mRotationDiff = mRotationDiff.unaryMinus().rotateBy(Rotation2d.fromRadians(Math.PI));
            }

            mStartTime = timestamp;
            if (Math.abs(mRotationDiff.getRadians()) < 0.1) {
                mDTheta = 0.0;
            } else {
                mDTheta = mRotationDiff.getRadians() / mTotalTime;
            }
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint sample_point;

        mHeadingSetpoint = new TimedRotation(
                mInitialHeading.rotateBy(mRotationDiff.times(Math.min(1.0,
                        (timestamp - mStartTime) / mTotalTime))));
        mCurrentState = current_state;
        if (!isDone()) {
            sample_point = mCurrentTrajectory.advance(mDt);
            // Compute error in robot frame
            mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state),
                    mPathSetpoint.state().poseMeters);
            mError = new Pose2d(mError.getTranslation(),
                    current_state.getRotation().unaryMinus()
                            .rotateBy(mHeadingSetpoint.state()));

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
                final Rotation2d rotation = mPathSetpoint.state().poseMeters.getRotation();

                // In field frame
                Translation2d chassis_v = new Translation2d(rotation.getCos() * velocity_m,
                        rotation.getSin() * velocity_m);
                // Convert to robot frame
                chassis_v = chassis_v.rotateBy(mHeadingSetpoint.state().unaryMinus());

                Twist2d chassis_twist = new Twist2d(chassis_v.getX(), chassis_v.getY(), mDTheta);

                // System.out.println(chassis_twist);

                ChassisSpeeds chassis_speeds = new ChassisSpeeds(
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
                    if (Math.abs(distance(current_state, previewQuantity) - 0.0) <= 0.01)
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
                log(mCurrentTrajectory);

                mOutput = updatePurePursuit(current_state, mDTheta);

            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new ChassisSpeeds();
        }
        return mOutput;
    }

    private void log(TrajectoryTimeIterator trajectory) {
        SmartDashboard.putString("Current Pose", mCurrentState.toString());
        SmartDashboard.putString("Last Pose",
                trajectory.trajectory().getLastPoint().state().state().poseMeters.toString());
        SmartDashboard.putNumber("Finished Traj?", mPathSetpoint.velocity());
    }

    public ChassisSpeeds update(TrajectoryTimeIterator trajectory, double timestamp, Pose2d current_state) {
        if (trajectory == null) {
            // System.out.println("YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
            return null;
        }

        if (trajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;

            mInitialHeading = trajectory.trajectory().getPoint(0).heading().state();
            var finalHeading = trajectory.trajectory().getLastPoint().heading().state();
            mTotalTime = trajectory.trajectory().getLastPoint().state().t() -
                    trajectory.trajectory().getPoint(0).state().t();
            // Interpolate heading
            mRotationDiff = finalHeading.rotateBy(mInitialHeading.unaryMinus());
            if (mRotationDiff.getRadians() > Math.PI) {
                mRotationDiff = mRotationDiff.unaryMinus().rotateBy(Rotation2d.fromRadians(Math.PI));
            }

            mStartTime = timestamp;
            if (Math.abs(mRotationDiff.getRadians()) < 0.1) {
                mDTheta = 0.0;
            } else {
                mDTheta = mRotationDiff.getRadians() / mTotalTime;
            }
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint sample_point;

        mHeadingSetpoint = new TimedRotation(
                mInitialHeading.rotateBy(mRotationDiff.times(Math.min(1.0,
                        (timestamp - mStartTime) / mTotalTime))));
        mCurrentState = current_state;
        if (!isDone()) {
            sample_point = trajectory.advance(mDt);
            // Compute error in robot frame
            mError = GeometryUtil.transformBy(GeometryUtil.inverse(current_state),
                    mPathSetpoint.state().poseMeters);
            mError = new Pose2d(mError.getTranslation(),
                    current_state.getRotation().unaryMinus()
                            .rotateBy(mHeadingSetpoint.state()));

            if (mFollowerType == FollowerType.PID) {
                mPathSetpoint = sample_point.state();

                // Generate feedforward voltages.
                final double velocity_m = Units.inchesToMeters(mPathSetpoint.velocity());
                final Rotation2d rotation = mPathSetpoint.state().poseMeters.getRotation();

                // In field frame
                Translation2d chassis_v = new Translation2d(rotation.getCos() * velocity_m,
                        rotation.getSin() * velocity_m);
                // Convert to robot frame
                chassis_v = chassis_v.rotateBy(mHeadingSetpoint.state().unaryMinus());

                Twist2d chassis_twist = new Twist2d(chassis_v.getX(), chassis_v.getY(), mDTheta);

                ChassisSpeeds chassis_speeds = new ChassisSpeeds(chassis_twist.dx, chassis_twist.dy, chassis_twist.dtheta);
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
                    if (Math.abs(distance(current_state, previewQuantity) - 0.0) <= 0.01)
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
                log(trajectory);

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

    public synchronized Translation2d getTranslationalError() {
        return new Translation2d(
                Units.inchesToMeters(mError.getTranslation().getX()),
                Units.inchesToMeters(mError.getTranslation().getY()));
    }

    public synchronized Rotation2d getHeadingError() {
        return mError.getRotation();
    }

    private double distance(Pose2d current_state, double additional_progress) {
        Pose2d pose = mCurrentTrajectory.preview(additional_progress).state().state().poseMeters;
        return GeometryUtil
                .norm(GeometryUtil.slog(GeometryUtil.transformBy(GeometryUtil.inverse(pose), current_state)));
    }

    public synchronized TimedPose getPathSetpoint() {
        return mPathSetpoint;
    }

    public synchronized TimedRotation getHeadingSetpoint() {
        return mHeadingSetpoint;
    }
}