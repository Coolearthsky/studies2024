package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;

public class TrajectoryPoint<S extends State<S>, T extends State<T>> {
    private final S state_;
    private final T heading_;
    private final int index_;

    public TrajectoryPoint(final S state, T heading, int index) {
        state_ = state;
        heading_ = heading;
        index_ = index;
    }

    public S state() {
        return state_;
    }

    public T heading() {
        return heading_;
    }

    public int index() {
        return index_;
    }
}
