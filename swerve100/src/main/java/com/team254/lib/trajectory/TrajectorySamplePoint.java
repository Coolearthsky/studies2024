package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;

public class TrajectorySamplePoint<S extends State<S>, T extends State<T>> {
    protected final S state_;
    protected final T heading_;
    protected final int index_floor_;
    protected final int index_ceil_;

    public TrajectorySamplePoint(final S state, final T heading, int index_floor, int index_ceil) {
        state_ = state;
        heading_ = heading;
        index_floor_ = index_floor;
        index_ceil_ = index_ceil;
    }

    public S state() {
        return state_;
    }

    public T heading() {
        return heading_;
    }

    public int index_floor() {
        return index_floor_;
    }

    public int index_ceil() {
        return index_ceil_;
    }
}
