package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;

public class IndexView<S extends State<S>, T extends State<T>> implements TrajectoryView<S, T> {
    private final Trajectory<S, T> trajectory;

    public IndexView(Trajectory<S, T> trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public TrajectorySamplePoint<S, T> sample(double index) {
        return trajectory.getInterpolated(index);
    }

    @Override
    public double last_interpolant() {
        return Math.max(0.0, trajectory.length() - 1);
    }

    @Override
    public double first_interpolant() {
        return 0.0;
    }

    @Override
    public Trajectory<S, T> trajectory() {
        return trajectory;
    }
}