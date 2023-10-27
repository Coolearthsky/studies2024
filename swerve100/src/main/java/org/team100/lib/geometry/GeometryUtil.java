package org.team100.lib.geometry;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;

public class GeometryUtil {

    public static final Rotation2d kRotationIdentity = new Rotation2d();
    public static final Rotation2d kPi = new Rotation2d(Math.PI);
    public static final Pose2d kPose2dIdentity = new Pose2d();
    public static final Translation2d kTranslation2dIdentity = new Translation2d();
    public static final Pose2dWithCurvature kPose2dWithCurvatureIdentity = new Pose2dWithCurvature();
    public static final Twist2d kTwist2dIdentity = new Twist2d(0.0, 0.0, 0.0);
    
    private GeometryUtil() {}
}
