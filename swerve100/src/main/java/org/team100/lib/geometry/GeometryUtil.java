package org.team100.lib.geometry;

import com.team254.lib.geometry.Pose2dState;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.geometry.Twist2dWrapper;
import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class GeometryUtil {

    public static final Rotation2dState kRotationIdentity = new Rotation2dState();
    public static final Rotation2dState kPi = new Rotation2dState(Math.PI);
    public static final Pose2dState kPose2dIdentity = new Pose2dState();
    public static final Translation2dState kTranslation2dIdentity = new Translation2dState();
    public static final Pose2dWithCurvature kPose2dWithCurvatureIdentity = new Pose2dWithCurvature();
    public static final Twist2dWrapper kTwist2dIdentity = new Twist2dWrapper(0.0, 0.0, 0.0);
    
    private GeometryUtil() {}

    public static boolean isParallel(Rotation2d a, Rotation2d b) {
        return Util.epsilonEquals(a.getRadians(), b.getRadians())
        || Util.epsilonEquals(a.getRadians(), Rotation2dState.WrapRadians(b.getRadians() + Math.PI));
    }

    public static Twist2d scale(Twist2d twist, double scale) {
        return new Twist2d(twist.dx*scale, twist.dy * scale, twist.dtheta*scale);
    }

    public static Pose2d transformBy(Pose2d a, Pose2d b) {
        return new Pose2d(a.getTranslation().plus(b.getTranslation().rotateBy(a.getRotation())),
        a.getRotation().rotateBy(b.getRotation()));
    }

    public static Pose2d inverse(Pose2d a) {
        Rotation2d rotation_inverted = a.getRotation().unaryMinus();
        return new Pose2d(a.getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);      
    }
}
