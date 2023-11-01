package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.spline.PoseWithCurvature;

public class CentripetalAccelerationConstraint implements TimingConstraint {
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(final Pose2dWithMotion state) {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithMotion state, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
