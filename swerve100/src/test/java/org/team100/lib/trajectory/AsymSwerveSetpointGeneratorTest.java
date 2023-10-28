package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinematics.SwerveKinematics;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;

import org.team100.lib.swerve.SwerveSetpoint;

import edu.wpi.first.math.geometry.Translation2d;

public class AsymSwerveSetpointGeneratorTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        // like 2023 comp bot
        double kTrackWidth = 0.491;
        double kWheelBase = 0.765;
        SwerveKinematics k = new SwerveKinematics(new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        org.team100.lib.swerve.SwerveDriveKinematics kinematics254 = k.as254();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics254);
        AsymSwerveSetpointGenerator.KinematicLimits limits = new AsymSwerveSetpointGenerator.KinematicLimits();
        limits.kMaxDriveVelocity = 5;
        limits.kMaxDriveAcceleration = 10;
        limits.kMaxSteeringVelocity = 5;
        org.team100.lib.swerve.ChassisSpeeds c254 = new org.team100.lib.swerve.ChassisSpeeds();
        c254.vxMetersPerSecond = 0;
        c254.vyMetersPerSecond = 0;
        c254.omegaRadiansPerSecond = 0;
        com.team254.lib.swerve.SwerveModuleState[] s254 = new com.team254.lib.swerve.SwerveModuleState[] {
                new com.team254.lib.swerve.SwerveModuleState(0, 0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity.get()),
                new com.team254.lib.swerve.SwerveModuleState(0, 0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity.get()),
                new com.team254.lib.swerve.SwerveModuleState(0, 0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity.get()),
                new com.team254.lib.swerve.SwerveModuleState(0, 0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity.get())
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(c254, s254);
        org.team100.lib.swerve.ChassisSpeeds cDesired254 = new org.team100.lib.swerve.ChassisSpeeds();
        cDesired254.vxMetersPerSecond = 10;
        cDesired254.vyMetersPerSecond = 10;
        cDesired254.omegaRadiansPerSecond = 10;
        double dt = 0.02;

        // initially it's not moving fast at all
        setpoint = swerveSetpointGenerator.generateSetpoint(limits, setpoint, cDesired254, dt);
        assertEquals(0, setpoint.mChassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.mChassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.mChassisSpeeds.omegaRadiansPerSecond, kDelta);

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = swerveSetpointGenerator.generateSetpoint(limits, setpoint, cDesired254, dt);
        }
        assertEquals(2.687, setpoint.mChassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(2.687, setpoint.mChassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(2.687, setpoint.mChassisSpeeds.omegaRadiansPerSecond, kDelta);

    }

}
