package org.team100.lib.motion.drivetrain;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.kinematics.SwerveKinematics;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal {
    private final Telemetry t = Telemetry.get();

    private final Experiments m_experiments;
    private final SpeedLimits m_speedLimits;
    private final SwerveDriveKinematics m_DriveKinematics;
    private final SwerveModuleCollectionInterface m_modules;

    private final double kWheelBase = .765;
    private final double kTrackWidth = .491;
    private final SwerveKinematics m_DriveKinematics2 = new SwerveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    private final AsymSwerveSetpointGenerator m_SwerveSetpointGenerator = new AsymSwerveSetpointGenerator(
            m_DriveKinematics2.as254());
    private AsymSwerveSetpointGenerator.KinematicLimits limits = new AsymSwerveSetpointGenerator.KinematicLimits();
    org.team100.lib.swerve.ChassisSpeeds c254 = new org.team100.lib.swerve.ChassisSpeeds();
    com.team254.lib.swerve.SwerveModuleState[] s254 = new com.team254.lib.swerve.SwerveModuleState[] {
            new com.team254.lib.swerve.SwerveModuleState(0, 0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity.get()),
            new com.team254.lib.swerve.SwerveModuleState(0, 0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity.get()),
            new com.team254.lib.swerve.SwerveModuleState(0, 0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity.get()),
            new com.team254.lib.swerve.SwerveModuleState(0, 0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity.get())
    };
    private SwerveSetpoint prevSetpoint = new SwerveSetpoint(c254, s254);

    // TODO: what is this?
    // private com.team254.lib.swerve.ChassisSpeeds desiredChassisSpeeds2 = new
    // com.team254.lib.swerve.ChassisSpeeds();

    public SwerveLocal(
            Experiments experiments,
            SpeedLimits speedLimits,
            SwerveDriveKinematics driveKinematics,
            SwerveModuleCollectionInterface modules) {
        m_experiments = experiments;
        m_speedLimits = speedLimits;
        m_DriveKinematics = driveKinematics;
        m_modules = modules;
        limits.kMaxDriveVelocity = 5;
        limits.kMaxDriveAcceleration = 1;
        limits.kMaxDriveDecceleration = 4;
        limits.kMaxSteeringVelocity = 5;
    }

    /**
     * Drives the modules to produce the target chassis speed.
     * 
     * @param targetChassisSpeeds speeds in robot coordinates.
     */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
        if (m_experiments.enabled(Experiment.UseSetpointGenerator)) {
            setChassisSpeedsWithSetpointGenerator(targetChassisSpeeds);
        } else {
            setChassisSpeedsNormally(targetChassisSpeeds);
        }
    }

    public void setChassisSpeeds254(org.team100.lib.swerve.ChassisSpeeds targetChassisSpeeds) {
        setChassisSpeedsNormally254(targetChassisSpeeds);
    }

    private void setChassisSpeedsNormally254(org.team100.lib.swerve.ChassisSpeeds targetChassisSpeeds) {
        t.log("/desired speed/x254", targetChassisSpeeds.vxMetersPerSecond);
        t.log("/desired speed/y254", targetChassisSpeeds.vyMetersPerSecond);
        t.log("/desired speed/theta254", targetChassisSpeeds.omegaRadiansPerSecond);

        com.team254.lib.swerve.SwerveModuleState[] swerveModuleStates254 = m_DriveKinematics2.as254()
                .toSwerveModuleStates(targetChassisSpeeds);
        Rotation2d thetafl = new Rotation2d(swerveModuleStates254[0].angle.getRadians());
        Rotation2d thetafr = new Rotation2d(swerveModuleStates254[1].angle.getRadians());
        Rotation2d thetabl = new Rotation2d(swerveModuleStates254[2].angle.getRadians());
        Rotation2d thetabr = new Rotation2d(swerveModuleStates254[3].angle.getRadians());
        SwerveModuleState fl = new SwerveModuleState(swerveModuleStates254[0].speedMetersPerSecond, thetafl);
        SwerveModuleState fr = new SwerveModuleState(swerveModuleStates254[1].speedMetersPerSecond, thetafr);
        SwerveModuleState bl = new SwerveModuleState(swerveModuleStates254[2].speedMetersPerSecond, thetabl);
        SwerveModuleState br = new SwerveModuleState(swerveModuleStates254[3].speedMetersPerSecond, thetabr);
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] { fl, fr, bl, br };

        t.log("/desired speed/front left", swerveModuleStates[0].speedMetersPerSecond);
        t.log("/desired speed/front right", swerveModuleStates[1].speedMetersPerSecond);
        t.log("/desired speed/back left", swerveModuleStates[2].speedMetersPerSecond);
        t.log("/desired speed/back right", swerveModuleStates[3].speedMetersPerSecond);

        setModuleStates(swerveModuleStates);
    }

    private void setChassisSpeedsNormally(ChassisSpeeds targetChassisSpeeds) {
        t.log("/desired speed/x", targetChassisSpeeds.vxMetersPerSecond);
        t.log("/desired speed/y", targetChassisSpeeds.vyMetersPerSecond);
        t.log("/desired speed/theta", targetChassisSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] targetModuleStates = m_DriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
        setModuleStates(targetModuleStates);
    }

    private void setChassisSpeedsWithSetpointGenerator(ChassisSpeeds targetChassisSpeeds2) {
        // public void driveMetersPerSec2(Twist2d twist, boolean fieldRelative) {
        // this is handled by the caller.
        // Rotation2d rotation2 = m_veering.correct(getPose().getRotation());

        // com.team254.lib.geometry.Rotation2d rotation254 = new
        // com.team254.lib.geometry.Rotation2d(rotation2.getRadians(), true);
        // desiredChassisSpeeds2 =
        // com.team254.lib.swerve.ChassisSpeeds.fromFieldRelativeSpeeds(twist.dx,
        // twist.dy, twist.dtheta, rotation254);
        // com.team254.lib.swerve.ChassisSpeeds targetChassisSpeeds = fieldRelative ?
        // desiredChassisSpeeds2
        // : new com.team254.lib.swerve.ChassisSpeeds(twist.dx, twist.dy, twist.dtheta);

        org.team100.lib.swerve.ChassisSpeeds targetChassisSpeeds = new org.team100.lib.swerve.ChassisSpeeds(
                targetChassisSpeeds2.vxMetersPerSecond, targetChassisSpeeds2.vyMetersPerSecond,
                targetChassisSpeeds2.omegaRadiansPerSecond);

        SwerveSetpoint setpoint = m_SwerveSetpointGenerator.generateSetpoint(limits, prevSetpoint, targetChassisSpeeds,
                .05);
        System.out.println(setpoint);
        prevSetpoint = setpoint;
        com.team254.lib.swerve.SwerveModuleState[] swerveModuleStates254 = m_DriveKinematics2.as254()
                .toSwerveModuleStates(setpoint.mChassisSpeeds);
        Rotation2d thetafl = new Rotation2d(swerveModuleStates254[0].angle.getRadians());
        Rotation2d thetafr = new Rotation2d(swerveModuleStates254[1].angle.getRadians());
        Rotation2d thetabl = new Rotation2d(swerveModuleStates254[2].angle.getRadians());
        Rotation2d thetabr = new Rotation2d(swerveModuleStates254[3].angle.getRadians());
        SwerveModuleState fl = new SwerveModuleState(swerveModuleStates254[0].speedMetersPerSecond, thetafl);
        SwerveModuleState fr = new SwerveModuleState(swerveModuleStates254[1].speedMetersPerSecond, thetafr);
        SwerveModuleState bl = new SwerveModuleState(swerveModuleStates254[2].speedMetersPerSecond, thetabl);
        SwerveModuleState br = new SwerveModuleState(swerveModuleStates254[3].speedMetersPerSecond, thetabr);
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] { fl, fr, bl, br };
        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the wheels to make an "X" pattern.
     * TODO: let the drivetrain decide to do this when it's stopped for awhile
     */
    public void defense() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(0, new Rotation2d(Math.PI / 4));
        states[1] = new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4));
        states[2] = new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4));
        states[3] = new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4));
        setModuleStates(states);
    }

    public SwerveModuleState[] states() {
        return m_modules.states();
    }

    /** The speed implied by the module states. */
    public ChassisSpeeds speeds() {
        SwerveModuleState[] states = states();
        return impliedSpeed(states);
    }

    public SwerveModulePosition[] positions() {
        return m_modules.positions();
    }

    public void stop() {
        m_modules.stop();
    }

    void test(double[][] desiredOutputs) {
        m_modules.test(desiredOutputs);
    }

    ///////////////////////////////////////////////////////////

    private void setModuleStates(SwerveModuleState[] targetModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, m_speedLimits.speedM_S);
        logImpliedChassisSpeeds(targetModuleStates);
        m_modules.setDesiredStates(targetModuleStates);
    }

    /**
     * Logs chassis speeds implied by the module settings. The difference from
     * the desired speed might be caused by, for example, desaturation.
     */
    private void logImpliedChassisSpeeds(SwerveModuleState[] actualModuleState) {
        ChassisSpeeds actualChassisSpeeds = impliedSpeed(actualModuleState);
        t.log("/actual speed/x", actualChassisSpeeds.vxMetersPerSecond);
        t.log("/actual speed/y", actualChassisSpeeds.vyMetersPerSecond);
        t.log("/actual speed/theta", actualChassisSpeeds.omegaRadiansPerSecond);
        t.log("/actual speed/moving", isMoving(actualChassisSpeeds));
    }

    /** The speed implied by the module states. */
    private ChassisSpeeds impliedSpeed(SwerveModuleState[] actualModuleState) {
        return m_DriveKinematics.toChassisSpeeds(
                actualModuleState[0],
                actualModuleState[1],
                actualModuleState[2],
                actualModuleState[3]);
    }

    private static boolean isMoving(ChassisSpeeds speeds) {
        return (speeds.vxMetersPerSecond >= 0.1
                || speeds.vyMetersPerSecond >= 0.1
                || speeds.omegaRadiansPerSecond >= 0.1);
    }
}
