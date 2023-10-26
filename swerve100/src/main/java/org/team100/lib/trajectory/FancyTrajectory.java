package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Telemetry;

import com.team254.frc2022.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

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
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(0));
        // these don't actually do anything.
        List<TimingConstraint<Pose2dWithCurvature>> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        // mMotionPlanner = new DriveMotionPlanner();
        boolean reversed = false;
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory = mMotionPlanner
                .generateTrajectory(
                        reversed,
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

        TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> iter = new TrajectoryIterator<>(
                new TimedView<>(trajectory));

        mMotionPlanner.reset();
        mMotionPlanner.setTrajectory(iter);
    }

    @Override
    public void execute() {
        final double now = Timer.getFPGATimestamp();

        Pose2d currentPose = new Pose2d(Units.metersToInches(m_robotDrive.getPose().getX()),
                Units.metersToInches(m_robotDrive.getPose().getY()),
                new Rotation2d(m_robotDrive.getPose().getRotation()));

        ChassisSpeeds output = mMotionPlanner.update(now, currentPose);

        t.log("/Fancy TrajectoryPose Error X",  mMotionPlanner.getTranslationalError().x());
        t.log("/Fancy Trajectory/Pose Error Y", mMotionPlanner.getTranslationalError().y());
        t.log("/Fancy Trajectory/Velocity Setpoint", mMotionPlanner.getVelocitySetpoint());

        m_robotDrive.setChassisSpeeds(output);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
