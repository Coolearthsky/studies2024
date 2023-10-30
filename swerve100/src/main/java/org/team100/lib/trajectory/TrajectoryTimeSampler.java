package org.team100.lib.trajectory;

/**
 * Allows sampling a trajectory by its schedule.
 */
public class TrajectoryTimeSampler {
    protected final Trajectory trajectory_;
    protected final double start_t_;
    protected final double end_t_;

    public TrajectoryTimeSampler(Trajectory trajectory) {
        trajectory_ = trajectory;
        start_t_ = trajectory_.getPoint(0).state().t();
        end_t_ = trajectory_.getPoint(trajectory_.length() - 1).state().t();
    }

    public double first_interpolant() {
        return start_t_;
    }

    public double last_interpolant() {
        return end_t_;
    }

    /**
     * @param t TODO what's the unit here? seconds?
     */
    public TrajectorySamplePoint sample(double t) {
        if (t >= end_t_) {
            TrajectoryPoint point = trajectory_.getPoint(trajectory_.length() - 1);
            return new TrajectorySamplePoint(point.state(), point.heading(), point.index(), point.index());
        }
        if (t <= start_t_) {
            TrajectoryPoint point = trajectory_.getPoint(0);
            return new TrajectorySamplePoint(point.state(), point.heading(), point.index(), point.index());
        }
        for (int i = 1; i < trajectory_.length(); ++i) {
            final TrajectoryPoint point = trajectory_.getPoint(i);
            if (point.state().t() >= t) {
                final TrajectoryPoint prev_s = trajectory_.getPoint(i - 1);
                if (Math.abs(point.state().t() - prev_s.state().t()) <= 1e-12) {
                    return new TrajectorySamplePoint(point.state(), point.heading(), point.index(), point.index());
                }
                return new TrajectorySamplePoint(
                        prev_s.state().interpolate2(point.state(),
                                (t - prev_s.state().t()) / (point.state().t() - prev_s.state().t())),
                        prev_s.heading().interpolate2(point.heading(),
                                (t - prev_s.heading().t()) / (point.heading().t() - prev_s.heading().t())),
                        i - 1, i);
            }
        }
        throw new RuntimeException();
    }

    public Trajectory trajectory() {
        return trajectory_;
    }
}
