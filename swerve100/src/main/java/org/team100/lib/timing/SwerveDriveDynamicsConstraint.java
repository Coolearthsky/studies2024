package org.team100.lib.timing;

import org.team100.lib.physics.SwerveDrive;
import org.team100.lib.timing.TimingConstraint.MinMaxAcceleration;

import edu.wpi.first.math.spline.PoseWithCurvature;

public class SwerveDriveDynamicsConstraint implements TimingConstraint {
    public static final double kMaxVelocityMetersPerSecond = 4.959668;
    public static final double kMaxDriveAcceleration = 1867 * 0.8; // m/s^2 tuned 2/18 practice bot

    protected final SwerveDrive drive_;
    protected final double abs_voltage_limit_;

    public SwerveDriveDynamicsConstraint(final SwerveDrive drive, double abs_voltage_limit) {
        drive_ = drive;
        abs_voltage_limit_ = abs_voltage_limit;
    }

    @Override
    public double getMaxVelocity(PoseWithCurvature state) {
        return kMaxVelocityMetersPerSecond / (1 + Math.abs(4.0 * state.curvatureRadPerMeter));// from 1323 TODO verify or fix
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(PoseWithCurvature state,
            double velocity) {
        return new MinMaxAcceleration(-kMaxDriveAcceleration, kMaxDriveAcceleration);
    }
}