package com.team254.lib.trajectory.timing;

import edu.wpi.first.math.spline.PoseWithCurvature;

public class CentripetalAccelerationConstraint implements TimingConstraint {
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(final PoseWithCurvature state) {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.curvatureRadPerMeter));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final PoseWithCurvature state, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
