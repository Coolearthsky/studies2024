package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.swerve.ChassisSpeeds;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimingConstraint;

import com.team254.frc2022.planners.DriveMotionPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FancyTrajectory extends Command {
    private final Telemetry t = Telemetry.get();
    private final SwerveDriveSubsystem m_robotDrive;
    private final DriveMotionPlanner mMotionPlanner;

    public FancyTrajectory(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        mMotionPlanner = new DriveMotionPlanner();
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        final double kMaxVel = 196;
        final double kMaxAccel = 196;
        final double kMaxVoltage = 9.0;

        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(80, 80, Rotation2d.fromDegrees(0)));
        // while turning 180
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(0),
                GeometryUtil.fromDegrees(0));
        // these don't actually do anything.
        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        // mMotionPlanner = new DriveMotionPlanner();
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory trajectory = mMotionPlanner
                .generateTrajectory(
                        waypoints,
                        headings,
                        constraints,
                        start_vel,
                        end_vel,
                        kMaxVel,
                        kMaxAccel,
                        kMaxVoltage);
        System.out.println(trajectory);
        System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        // assertEquals(10, trajectory.length());

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(new TrajectoryTimeSampler(trajectory));

        mMotionPlanner.reset();
        mMotionPlanner.setTrajectory(iter);
    }

    @Override
    public void execute() {
        final double now = Timer.getFPGATimestamp();

        // TODO: remove this meters/inches stuff
        Pose2d currentPose = new Pose2d(Units.metersToInches(m_robotDrive.getPose().getX()),
                Units.metersToInches(m_robotDrive.getPose().getY()),
                m_robotDrive.getPose().getRotation());

        ChassisSpeeds output = mMotionPlanner.update(now, currentPose);

        t.log("/Fancy TrajectoryPose Error X",  mMotionPlanner.getTranslationalError().getX());
        t.log("/Fancy Trajectory/Pose Error Y", mMotionPlanner.getTranslationalError().getY());
        t.log("/Fancy Trajectory/Velocity Setpoint", mMotionPlanner.getVelocitySetpoint());

        m_robotDrive.setChassisSpeeds(output);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
