package org.team100.lib.geometry;

import com.team254.lib.geometry.Pose2dState;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.geometry.Twist2dWrapper;
import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

public class GeometryUtil {

    public static final Rotation2dState kRotationIdentity = new Rotation2dState();
    public static final Rotation2dState kPi = new Rotation2dState(Math.PI);
    public static final Pose2dState kPose2dIdentity = new Pose2dState();
    public static final Translation2dState kTranslation2dIdentity = new Translation2dState();
    public static final Pose2dWithCurvature kPose2dWithCurvatureIdentity = new Pose2dWithCurvature();
    public static final Twist2dWrapper kTwist2dIdentity = new Twist2dWrapper(0.0, 0.0, 0.0);

    private GeometryUtil() {
    }


    public static Twist2d scale(Twist2d twist, double scale) {
        return new Twist2d(twist.dx * scale, twist.dy * scale, twist.dtheta * scale);
    }

    public static Pose2d transformBy(Pose2d a, Pose2d b) {
        return a.transformBy(new Transform2d(b.getTranslation(), b.getRotation()));
    }

    public static Pose2d inverse(Pose2d a) {
        Rotation2d rotation_inverted = a.getRotation().unaryMinus();
        return new Pose2d(a.getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);
    }

    public static Twist2dWrapper slog(final Pose2dState transform) {
        Pose2dState base = new Pose2dState();
        return new Twist2dWrapper(base.pose2d.log(transform.pose2d));
    }

    public static Pose2dState sexp(final Twist2dWrapper delta) {
        return new Pose2dState(new Pose2dState().pose2d.exp(delta));
    }

    public static Pose2dState fromRotation(final Rotation2dState rotation) {
        return new Pose2dState(new Translation2dState(), rotation);
    }

    public static Pose2dState fromTranslation(final Translation2dState translation) {
        return new Pose2dState(translation, new Rotation2dState());
    }

    public static Rotation2dState fromRadians(double angle_radians) {
        return new Rotation2dState(angle_radians);
    }

    public static Rotation2dState fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public static double WrapRadians(double radians) {
        final double k2Pi = 2.0 * Math.PI;
        radians = radians % k2Pi;
        radians = (radians + k2Pi) % k2Pi;
        if (radians > Math.PI)
            radians -= k2Pi;
        return radians;
    }

    public static Pose2d inverse(Pose2dState a) {
        return inverse(a.get());
    }

    public static boolean isColinear(Pose2d a, final Pose2d other) {
        if (!GeometryUtil.isParallel(a.getRotation(), other.getRotation()))
            return false;
        final Twist2dWrapper twist = slog(
                new Pose2dState(transformBy(inverse(a), other)));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }
    
    // note parallel also means antiparallel.
    public static boolean isParallel(Rotation2d a, Rotation2d b) {
        return Util.epsilonEquals(a.getRadians(), b.getRadians())
                || Util.epsilonEquals(a.getRadians(), WrapRadians(b.getRadians() + Math.PI));
    }
}
