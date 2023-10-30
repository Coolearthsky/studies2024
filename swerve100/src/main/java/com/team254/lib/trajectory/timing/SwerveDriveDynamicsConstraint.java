package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.physics.SwerveDrive;

public class SwerveDriveDynamicsConstraint implements TimingConstraint<Pose2dWithCurvature> {
    public static final double kMaxVelocityMetersPerSecond = 4.959668;
    public static final double kMaxDriveAcceleration = 1867 * 0.8;   // m/s^2 tuned 2/18 practice bot


    protected final SwerveDrive drive_;
    protected final double abs_voltage_limit_;

    public SwerveDriveDynamicsConstraint(final SwerveDrive drive, double abs_voltage_limit) {
        drive_ = drive;
        abs_voltage_limit_ = abs_voltage_limit;
    }

    @Override
    public double getMaxVelocity(Pose2dWithCurvature state) {
        return kMaxVelocityMetersPerSecond / (1 + Math.abs(4.0*state.getCurvature()));// from 1323 TODO verify or fix
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithCurvature state,
                                                    double velocity) {
        return new MinMaxAcceleration(-kMaxDriveAcceleration, kMaxDriveAcceleration);
    }
}