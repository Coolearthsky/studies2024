package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;

public class IndexView<S extends State<S>, T extends State<T>> {
    private final Trajectory<S, T> trajectory;

    public IndexView(Trajectory<S, T> trajectory) {
        this.trajectory = trajectory;
    }

    public TrajectorySamplePoint<S, T> sample(double index) {
        return trajectory.getInterpolated(index);
    }

    public double last_interpolant() {
        return Math.max(0.0, trajectory.length() - 1);
    }

    public double first_interpolant() {
        return 0.0;
    }

    public Trajectory<S, T> trajectory() {
        return trajectory;
    }
}