package frc.robot.arm;

import frc.robot.Robot;
import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmTrajectory extends Command {
    public static class Config {
        public double oscillatorFrequencyHz = 2;
        /** amplitude (each way) of oscillation in encoder units */
        public double oscillatorScale = 0.025;
        /** start oscillating when this close to the target. */
        public double oscillatorZone = 0.1;
        public TrajectoryConfig safeTrajectory = new TrajectoryConfig(9, 1.5);
        public TrajectoryConfig normalTrajectory = new TrajectoryConfig(12, 2);
    }
    private final Config m_config = new Config();
    private final Robot m_robot;
    // private final ArmPosition m_position;
    // private final boolean m_oscillate;
    private final ArmKinematics m_kinematics;
    private final Timer m_timer;
    private final Translation2d m_set;
    private final DoublePublisher measurmentX;
    private final DoublePublisher measurmentY;
    private final DoublePublisher setpointUpper;
    private final DoublePublisher setpointLower;

    private Trajectory m_trajectory;

    /**
     * Go to the specified position and optionally oscillate when you get there.
     */
    public ArmTrajectory(Robot robot, Translation2d set, ArmKinematics kinematics) {
        m_set = set;
        m_robot = robot;
        m_kinematics = kinematics;
        // m_position = position;
        // m_oscillate = oscillate;
        m_timer = new Timer();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        measurmentX = inst.getTable("Arm Trajec").getDoubleTopic("measurmentX").publish();
        measurmentY = inst.getTable("Arm Trajec").getDoubleTopic("measurmentY").publish();
        setpointUpper = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Upper").publish();
        setpointLower = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Lower").publish();

        // addRequirements(m_robot);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        final TrajectoryConfig trajectoryConfig;
        // if (m_position == ArmPosition.SAFE) {
        //     trajectoryConfig = m_config.safeTrajectory;
        //     m_arm.setControlSafe();
        // } else {
            trajectoryConfig = m_config.normalTrajectory;
            // m_arm.setControlNormal();
        // }
        System.out.println("Initialize");
        m_trajectory = new ArmTrajectories(trajectoryConfig).makeTrajectory(
            m_kinematics.forward(m_robot.getMeasurement()),m_set);
           
    }

    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        ArmAngles measurement = m_robot.getMeasurement();
        double currentUpper = measurement.th2;
        double currentLower = measurement.th1;
        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);
        double desiredXPos = desiredState.poseMeters.getX();
        double desiredYPos  = desiredState.poseMeters.getY();
        double desiredVecloity = desiredState.velocityMetersPerSecond;
        double theta = desiredState.poseMeters.getRotation().getRadians();
        double desiredXVelocity = desiredVecloity*Math.cos(theta);
        double desiredYVelocity = desiredVecloity*Math.cos(90-theta);
        Translation2d reference = new Translation2d(desiredXPos, desiredYPos);
        m_robot.setReference(m_kinematics.inverse(reference));
        measurmentX.set(currentUpper);
        measurmentY.set(currentLower);
        setpointUpper.set(desiredYPos);
        setpointLower.set(desiredXPos);
    }

    @Override
    public void end(boolean interrupted) {
        // m_arm.setControlNormal();
        m_robot.setReference(m_robot.getMeasurement());
    }

    @Override
    public boolean isFinished() {
        double th1 = m_kinematics.inverse(m_set).th1;
        double th2 = m_kinematics.inverse(m_set).th2;
        double lowerError = m_robot.getMeasurement().th1 - th1;
        double upperError = m_robot.getMeasurement().th2 - th2;
        if (Math.abs(upperError) < 0.02 && Math.abs(lowerError) < 0.02) {
            System.out.println("ENDDDDDDDDDDDDDDDDDDDDDD");
            return true;
        }
        return false;
    }

    // private double oscillator(double timeSec) {
    //     return m_config.oscillatorScale * Math.sin(2 * Math.PI * m_config.oscillatorFrequencyHz * timeSec);
    // }

}
