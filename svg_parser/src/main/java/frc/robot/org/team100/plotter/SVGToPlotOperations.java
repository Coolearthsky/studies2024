package frc.robot.org.team100.plotter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;

/**
 * A collection of plot operations made from SVG path operators.
 */
public class SVGToPlotOperations {
    private static final double ABS_MAX = 1000;

    private final double xScale;
    private final double yScale;
    private final List<Operation> operations;

    private double currentX = 0;
    private double currentY = 0;
    private Double initialX = null;
    private Double initialY = null;

    /**
     * Note that WPILib spline generation makes a scale assumption, and fails if the
     * step size is larger than some absolute scale, don't give it
     * anything more than like 1000?
     */
    public SVGToPlotOperations(double xScale, double yScale) {
        this.xScale = xScale;
        this.yScale = yScale;
        operations = new ArrayList<>();
    }

    public void move(double rawX, double rawY) {
        double x = xScale * rawX;
        double y = yScale * rawY;
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX)
            throw new IllegalArgumentException(String.format("args too big %f %f", rawX, rawY));
        moveScaled(x, y);
    }

    private void moveScaled(double x, double y) {
        lineScaled(x, y, false);
        this.initialX = x;
        this.initialY = y;
    }

    public void line(double rawX, double rawY) {
        double x = xScale * rawX;
        double y = yScale * rawY;
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX)
            throw new IllegalArgumentException(String.format("args too big %f %f", rawX, rawY));

        lineScaled(x, y, true);
    }

    /** Make a trajectory with just a start and end. */
    private Trajectory shortTrajectory(Pose2d startPose, Pose2d endPose) {
        // TODO: use nonzero velocities
        Trajectory.State start = new Trajectory.State(0, 0, 0, startPose, 0);
        Trajectory.State end = new Trajectory.State(0.1, 0, 0, endPose, 0);
        return new Trajectory(List.of(start, end));
    }

    private void lineScaled(double x, double y, boolean penDown) {
        // a line is a curve with control points in the right places.

        // so actually all that matters is the direction.

        double scale = 0.1;

        double x0dot = x - this.currentX;
        double y0dot = y - this.currentY;
        double x1dot = x0dot;
        double y1dot = y0dot;

        curveHermite(x, y, scale * x0dot, scale * y0dot, scale * x1dot, scale * y1dot, penDown);
    }

    public void curve(double rawX, double rawY, double rawX1, double rawY1, double rawX2, double rawY2) {
        double x = xScale * rawX;
        double y = yScale * rawY;
        double x1 = xScale * rawX1;
        double y1 = yScale * rawY1;
        double x2 = xScale * rawX2;
        double y2 = yScale * rawY2;
        if (Math.abs(x) > ABS_MAX || Math.abs(y) > ABS_MAX ||
                Math.abs(x1) > ABS_MAX || Math.abs(y1) > ABS_MAX ||
                Math.abs(x2) > ABS_MAX || Math.abs(y2) > ABS_MAX)
            throw new IllegalArgumentException(String.format(
                    "args too big %f %f %f %f %f %f", rawX, rawY, rawX1, rawY1, rawX2, rawY2));
        curveScaled(x, y, x1, y1, x2, y2, true);
    }

    private void curveScaled(double x, double y, double x1, double y1, double x2, double y2, boolean penDown) {
        // convert the control points into Hermite derivatives
        double x0dot = (x1 - this.currentX) / 3;
        double y0dot = (y1 - this.currentY) / 3;
        double x1dot = (x - x2) / 3;
        double y1dot = (y - y2) / 3;

        curveHermite(x, y, x0dot, y0dot, x1dot, y1dot, penDown);
    }

    /** any curve or line */
    private void curveHermite(
            double x, double y,
            double x0dot, double y0dot,
            double x1dot, double y1dot,
            boolean penDown) {

        System.out.printf("hermite curve %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n", this.currentX, this.currentY, x, y, x0dot, y0dot, x1dot, y1dot);

        Rotation2d rot = new Rotation2d(x - this.currentX, y - this.currentY);
        Pose2d start = new Pose2d(this.currentX, this.currentY, rot);
        Pose2d end = new Pose2d(x, y, rot);
        // this constant has uh no actual meaning
        if (end.minus(start).getTranslation().getNorm() < 0.2) {
            // too short, trajectory generator barfs
            System.out.println("using short trajectory");
            Trajectory trajectory = shortTrajectory(start, end);
            operations.add(new Operation(penDown, trajectory));

            this.currentX = x;
            this.currentY = y;
            return;
        }

        TrajectoryConfig config = new TrajectoryConfig(10, 0.1);
        // i have no idea why this is 9
        final double scale = 9;

        double[] xInitialControlVector = new double[] { this.currentX, scale * x0dot };
        double[] yInitialControlVector = new double[] { this.currentY, scale * y0dot };
        double[] xFinalControlVector = new double[] { x, scale * x1dot };
        double[] yFinalControlVector = new double[] { y, scale * y1dot };

        System.out.printf("curve %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                x, y, x0dot, y0dot, x1dot, y1dot);

        CubicHermiteSpline spline = new CubicHermiteSpline(
                xInitialControlVector,
                xFinalControlVector,
                yInitialControlVector,
                yFinalControlVector);

        Spline[] splines = new Spline[] { spline };
        List<PoseWithCurvature> points = TrajectoryGenerator.splinePointsFromSplines(splines);

        Trajectory trajectory = TrajectoryParameterizer.timeParameterizeTrajectory(
                points,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.isReversed());

        operations.add(new Operation(penDown, trajectory));

        this.currentX = x;
        this.currentY = y;
    }

    public void close() {
        if (initialX == null || initialY == null)
            throw new IllegalStateException("can't close a path before starting one");
        lineScaled(initialX, initialY, true);
    }

    public List<Operation> getOperations() {
        return operations;
    }

}
