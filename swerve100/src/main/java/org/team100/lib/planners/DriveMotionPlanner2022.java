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
import org.team100.lib.telemetry.Telemetry;
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

/**
 * This is derived from 254's 2022 version.
 * It seems unfinished.  See the 2023 version.
 */
public class DriveMotionPlanner2022 {
    public enum FollowerType {
        FEEDFORWARD_ONLY,
        PID,
        PURE_PURSUIT
    }

    private final Telemetry t = Telemetry.get();

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
    private static final double defaultCook = 0.4;

    private final FollowerType mFollowerType;
    private final SwerveDrive mModel;

    private boolean useDefaultCook = true;

    private TrajectoryTimeIterator mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTimeS = Double.POSITIVE_INFINITY;
    public TimedPose mLastPathSetpointM = null;
    public TimedPose mPathSetpointM = new TimedPose(new PoseWithCurvature());
    public Rotation2d mHeadingSetpoint = null;
    public TimedRotation mLastHeadingSetpoint = new TimedRotation(new Rotation2d());

    private double mVelocityM_S = 0;
    private Pose2d mErrorM = GeometryUtil.kPose2dIdentity;

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

    public DriveMotionPlanner2022(FollowerType type) {
        mFollowerType = type;
        mModel = new SwerveDrive(
                0.0942 / 2,
                0.464 / 2.0 * kTrackScrubFactor);
    }

    public void setTrajectory(final TrajectoryTimeIterator trajectory) {
        mCurrentTrajectory = trajectory;
        mPathSetpointM = trajectory.getState();
        mHeadingSetpoint = trajectory.getHeading().state();
        mLastHeadingSetpoint = null;
        mLastPathSetpointM = null;
        useDefaultCook = true;
        mSpeedLookahead = new Lookahead(kAdaptivePathMinLookaheadDistance,
                kAdaptivePathMaxLookaheadDistance, 0.0,
                kMaxVelocityMetersPerSecond);
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
        mLastPathSetpointM = null;
        mOutput = new ChassisSpeeds();
        mLastTimeS = Double.POSITIVE_INFINITY;
    }

    public Trajectory generateTrajectory(
            final List<Pose2d> waypointsM,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double startVelM_S,
            double endVelM_S,
            double maxVelM_S,
            double maxAccelM_S_S,
            double max_voltage) {

        // Create a path from splines.

        List<QuinticHermiteSpline> splines = new ArrayList<>(waypointsM.size() - 1);
        for (int i1 = 1; i1 < waypointsM.size(); ++i1) {
            splines.add(new QuinticHermiteSpline(waypointsM.get(i1 - 1), waypointsM.get(i1)));
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
                startVelM_S, endVelM_S, maxVelM_S, maxAccelM_S_S);
    }

    /**
     * Adds error to chassisSpeeds.
     * 
     * Why?
     * 
     * TODO: this seems wrong
     */
    protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
        System.out.println("update chassis speeds " + chassisSpeeds);
        System.out.println("update error " + mErrorM);
        // Feedback on longitudinal error (distance).
        final double kPathk = 0.6;
        // final double kPathk = 0.15;
        // final double kPathk = Math.hypot(chassisSpeeds.vxMetersPerSecond,
        // chassisSpeeds.vyMetersPerSecond);
        final double kPathKTheta = 0.3;

        // TODO: what does this math mean?
        chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond + kPathk * mErrorM.getTranslation().getX();
        chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond + kPathk * mErrorM.getTranslation().getY();
        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
                + kPathKTheta * mErrorM.getRotation().getRadians();
        System.out.println("update final chassis speeds " + chassisSpeeds);
        return chassisSpeeds;
    }

    protected ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond) {
        double lookahead_time = kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedPose lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();

        PoseWithCurvature lookstate = lookahead_state.state();
        PoseWithCurvature setpointstate = mPathSetpointM.state();
        final PoseWithCurvature other1 = lookstate;
        double actual_lookahead_distance = GeometryUtil.norm(
                GeometryUtil.slog(
                        GeometryUtil.transformBy(
                                GeometryUtil.inverse(setpointstate.poseMeters),
                                other1.poseMeters)));

        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mPathSetpointM.velocity())
                + kAdaptiveErrorLookaheadCoefficient * mErrorM.getTranslation().getNorm();

        t.log("/planner/Adaptive Lookahead", adaptive_lookahead_distance);

        // Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();

            PoseWithCurvature state = lookahead_state.state();
            PoseWithCurvature state2 = mPathSetpointM.state();
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

        t.log("/planner/Path X", lookahead_state.state().poseMeters.getTranslation().getX());
        t.log("/planner/Path Y", lookahead_state.state().poseMeters.getTranslation().getY());
        t.log("/planner/Path Velocity", lookahead_state.velocity());

        // Find the vector between robot's current position and the lookahead state
        Translation2d lookaheadTranslation = lookahead_state.state().poseMeters.getTranslation()
                .minus(current_state.getTranslation());

        // Set the steering direction as the direction of the vector
        Rotation2d steeringDirection = lookaheadTranslation.getAngle();
        t.log("/planner/Steering Direction", steeringDirection.toString());

        // Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(GeometryUtil.inverse(current_state).getRotation());
        t.log("/planner/Steering Direction After Rotation", steeringDirection.toString());

        // Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeedFrac = Math.abs(mPathSetpointM.velocity()) / kMaxVelocityMetersPerSecond;

        // The Default Cook is the minimum speed to use. So if a feedforward speed is
        // less than defaultCook, the robot will drive at the defaultCook speed
        if (normalizedSpeedFrac > defaultCook || mPathSetpointM.t() > (mCurrentTrajectoryLength / 2.0)) {
            useDefaultCook = false;
        }
        if (useDefaultCook) {
            normalizedSpeedFrac = defaultCook;
        }

        // Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate
        // (Vx, Vy) in Robot Frame
        final Translation2d steeringVector = new Translation2d(
                steeringDirection.getCos() * normalizedSpeedFrac,
                steeringDirection.getSin() * normalizedSpeedFrac);
        t.log("/planner/Steering Vector", steeringVector.toString());
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                steeringVector.getX() * kMaxVelocityMetersPerSecond,
                steeringVector.getY() * kMaxVelocityMetersPerSecond, feedforwardOmegaRadiansPerSecond);

        // Use the P-Controller for To Follow the Time-Parametrized Heading
        final double kPathKTheta = 0.3;

        chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
                + kPathKTheta * mErrorM.getRotation().getRadians();
        return chassisSpeeds;
    }

    public ChassisSpeeds update(double timestampS, Pose2d current_state) {
        System.out.println("update ==================");
        t.log("/planner/update/timestamp s", timestampS);
        t.log("/planner/update/current state", current_state);

        if (mCurrentTrajectory == null) {
            return null;
        }

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTimeS)) {
            System.out.println("initialize trajectory");
            mLastTimeS = timestampS;

            mInitialHeading = mCurrentTrajectory.trajectory().getPoint(0).heading().state();
            Rotation2d finalHeading = mCurrentTrajectory.trajectory().getLastPoint().heading().state();
            mTotalTime = mCurrentTrajectory.trajectory().getLastPoint().state().t() -
                    mCurrentTrajectory.trajectory().getPoint(0).state().t();
            t.log("/planner/total time", mTotalTime);


            // Interpolate heading
            mRotationDiff = finalHeading.rotateBy(mInitialHeading.unaryMinus());
            if (mRotationDiff.getRadians() > Math.PI) {
                mRotationDiff = mRotationDiff.unaryMinus().rotateBy(Rotation2d.fromRadians(Math.PI));
            }

            mStartTime = timestampS;
            if (Math.abs(mRotationDiff.getRadians()) < 0.1) {
                System.out.println("no rotation");
                mDTheta = 0.0;
            } else {
                System.out.println("linear interpolation of rotation!");
                mDTheta = mRotationDiff.getRadians() / mTotalTime;
            }
        }
        t.log("/planner/dtheta", mDTheta);
        t.log("/planner/rotation diff", mRotationDiff);

        double dtSec = timestampS - mLastTimeS;
        t.log("/planner/dt s", dtSec);

        mLastTimeS = timestampS;

        // this is robot heading
        // ****************************8
        // TODO: i think this is ignoring the trajectory timed rotation!

        mHeadingSetpoint = mInitialHeading.rotateBy(mRotationDiff.times(Math.min(1.0,
                (timestampS - mStartTime) / mTotalTime)));

        t.log("/planner/heading setpoint", mHeadingSetpoint);

        mCurrentState = current_state;
        if (!isDone()) {

            TrajectorySamplePoint sample_point = mCurrentTrajectory.advance(dtSec);
            t.log("/planner/sample", sample_point);

            // Compute error in robot frame
            // why in robot frame?
            // this is the *previous* setpoint. is that correct?
            mErrorM = GeometryUtil.transformBy(GeometryUtil.inverse(current_state),
                    mPathSetpointM.state().poseMeters);
            mErrorM = new Pose2d(mErrorM.getTranslation(),
                    current_state.getRotation().unaryMinus()
                            .rotateBy(mHeadingSetpoint));
            t.log("/planner/error", mErrorM);

            if (mFollowerType == FollowerType.PID) {
                System.out.println("using follower type PID");

                mPathSetpointM = sample_point.state();
                t.log("/planner/path setpoint", mPathSetpointM);

                // Generate feedforward voltages.

                final double velocity_m = mPathSetpointM.velocity();
                t.log("/planner/velocity m", velocity_m);

                // this is the spline angle not the robot heading
                final Rotation2d rotation = mPathSetpointM.state().poseMeters.getRotation();
                t.log("/planner/rotation rad", rotation.getRadians());

                // In field frame
                // TODO what does this mean?
                Translation2d chassis_v = new Translation2d(rotation.getCos() * velocity_m,
                        rotation.getSin() * velocity_m);
                t.log("/planner/chassis v field frame", chassis_v);

                // Convert to robot frame
                chassis_v = chassis_v.rotateBy(mHeadingSetpoint.unaryMinus());
                t.log("/planner/chassis v robot frame", chassis_v);

                Twist2d chassis_twist = new Twist2d(chassis_v.getX(), chassis_v.getY(), mDTheta);
                t.log("/planner/chassis twist", chassis_twist);

                ChassisSpeeds chassis_speeds = new ChassisSpeeds(
                        chassis_twist.dx, chassis_twist.dy, chassis_twist.dtheta);
                // PID is in robot frame

                mOutput = updatePIDChassis(chassis_speeds);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                System.out.println("using follower type PURE_PURSUIT");

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
                mPathSetpointM = sample_point.state();

                mOutput = updatePurePursuit(current_state, mDTheta);

            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new ChassisSpeeds();
        }
        log(mCurrentTrajectory);
        t.log("/planner update/output", mOutput);
        System.out.println("update done ========");
        return mOutput;
    }

    private void log(TrajectoryTimeIterator trajectory) {
        t.log("/planner/Current Pose", mCurrentState.toString());
        t.log("/planner/Last Pose", trajectory.trajectory().getLastPoint().state().state().poseMeters.toString());
        t.log("/planner/Path setpoint velocity", mPathSetpointM.velocity());
        t.log("/planner/Pose Error X", mErrorM.getTranslation().getX());
        t.log("/planner/Pose Error Y", mErrorM.getTranslation().getY());
        t.log("/planner/Velocity Setpoint", mVelocityM_S);

    }

    public boolean isDone() {
        return mCurrentTrajectory != null && (mCurrentTrajectory.isDone());
    }

    private double distance(Pose2d current_state, double additional_progress) {
        Pose2d pose = mCurrentTrajectory.preview(additional_progress).state().state().poseMeters;
        return GeometryUtil
                .norm(GeometryUtil.slog(GeometryUtil.transformBy(GeometryUtil.inverse(pose), current_state)));
    }

    // for testing
    public TimedPose getPathSetpoint() {
        return mPathSetpointM;
    }

    // for testing
    public Rotation2d getHeadingSetpoint() {
        return mHeadingSetpoint;
    }

    // for testing
    public Translation2d getTranslationalErrorM() {
        return mErrorM.getTranslation();
    }

    // for testing
    public Rotation2d getHeadingError() {
        return mErrorM.getRotation();
    }
}