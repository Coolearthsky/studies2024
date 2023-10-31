package org.team100.lib.motion.drivetrain.kinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Wraps both WPI and 254 kinematics */
public class SwerveKinematics {
    private final SwerveDriveKinematics kWPI;
    private final SwerveDriveKinematics k254;

    public edu.wpi.first.math.kinematics.SwerveDriveKinematics asWPI() {
        return kWPI;
    }

    public SwerveDriveKinematics as254() {
        return k254;
    }

    public SwerveKinematics(Translation2d... wheelsMeters) {
        kWPI = new SwerveDriveKinematics(wheelsMeters);
        Translation2d[] w254 = new Translation2d[wheelsMeters.length];

        for (int i = 0; i < wheelsMeters.length; ++i) {
            Translation2d t = wheelsMeters[i];
            w254[i] = new Translation2d(t.getX(), t.getY());
        }
        k254 = new SwerveDriveKinematics(w254);

    }

}
