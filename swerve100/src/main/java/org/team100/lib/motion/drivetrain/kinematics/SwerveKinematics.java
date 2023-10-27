package org.team100.lib.motion.drivetrain.kinematics;

import edu.wpi.first.math.geometry.Translation2d;

/** Wraps both WPI and 254 kinematics */
public class SwerveKinematics {
    private final edu.wpi.first.math.kinematics.SwerveDriveKinematics kWPI;
    private final org.team100.lib.swerve.SwerveDriveKinematics k254;

    public edu.wpi.first.math.kinematics.SwerveDriveKinematics asWPI() {
        return kWPI;
    }

    public org.team100.lib.swerve.SwerveDriveKinematics as254() {
        return k254;
    }

    public SwerveKinematics(Translation2d... wheelsMeters) {
        kWPI = new edu.wpi.first.math.kinematics.SwerveDriveKinematics(wheelsMeters);
        com.team254.lib.geometry.Translation2dState[] w254 = new com.team254.lib.geometry.Translation2dState[wheelsMeters.length];

        for (int i = 0; i < wheelsMeters.length; ++i) {
            Translation2d t = wheelsMeters[i];
            w254[i] = new com.team254.lib.geometry.Translation2dState(t.getX(), t.getY());
        }
        k254 = new org.team100.lib.swerve.SwerveDriveKinematics(w254);

    }

}
