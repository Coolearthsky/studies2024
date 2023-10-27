package org.team100.lib.geometry;

import com.team254.lib.geometry.Pose2dState;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.geometry.Twist2dWrapper;

public class GeometryUtil {

    public static final Rotation2dState kRotationIdentity = new Rotation2dState();
    public static final Rotation2dState kPi = new Rotation2dState(Math.PI);
    public static final Pose2dState kPose2dIdentity = new Pose2dState();
    public static final Translation2dState kTranslation2dIdentity = new Translation2dState();
    public static final Pose2dWithCurvature kPose2dWithCurvatureIdentity = new Pose2dWithCurvature();
    public static final Twist2dWrapper kTwist2dIdentity = new Twist2dWrapper(0.0, 0.0, 0.0);
    
    private GeometryUtil() {}
}
