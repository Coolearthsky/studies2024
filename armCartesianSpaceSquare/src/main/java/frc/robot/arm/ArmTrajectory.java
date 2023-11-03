package frc.robot.arm;

import frc.robot.Robot;
import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmTrajectory extends Command {
    public static class Config {
        public double softStop = -0.594938;
        public double kUpperArmLengthM = 0.92;
        public double kLowerArmLengthM = 0.93;
        public double filterTimeConstantS = 0.06; // TODO: tune the time constant
        public double filterPeriodS = 0.02;
        public double safeP = 2.5;
        public double safeI = 0;
        public double safeD = 0;
        public double normalLowerP = 2;
        public double normalLowerI = 0;
        public double normalLowerD = 0.1;
        public double normalUpperP = 2;
        public double normalUpperI = 0;
        public double normalUpperD = 0.05;
        public double tolerance = 0.001;
        public double oscillatorFrequencyHz = 2;
        /** amplitude (each way) of oscillation in encoder units */
        public double oscillatorScale = 0.025;
        /** start oscillating when this close to the target. */
        public double oscillatorZone = 0.1;
        public TrajectoryConfig safeTrajectory = new TrajectoryConfig(9, 1.5);
        public TrajectoryConfig normalTrajectory = new TrajectoryConfig(1, 1);
    }
    private final Config m_config = new Config();
    private final Robot m_robot;
    private final double m_startAngle;
    private final double m_endAngle;
    // private final ArmPosition m_position;
    // private final boolean m_oscillate;
    private final ArmKinematics m_kinematics;
    private final Timer m_timer;
    private final Translation2d m_set;
    private final PIDController m_lowerController; 
    private final PIDController m_upperController; 
    private final DoublePublisher measurmentX;
    private final DoublePublisher measurmentY;
    private final DoublePublisher setpointUpper;
    private final DoublePublisher setpointLower;

    private Trajectory m_trajectory;

    /**
     * Go to the specified position and optionally oscillate when you get there.
     * Units for angles are degrees
     */
    public ArmTrajectory(Robot robot, Translation2d set, ArmKinematics kinematics, double startAngle, double endAngle) {
        m_set = set;
        m_robot = robot;
        m_kinematics = kinematics;
        m_endAngle = endAngle;
        m_startAngle = startAngle;
        // m_position = position;
        // m_oscillate = oscillate;
        m_timer = new Timer();
        m_lowerController = new PIDController(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperController = new PIDController(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        measurmentX = inst.getTable("Arm Trajec").getDoubleTopic("measurmentX").publish();
        measurmentY = inst.getTable("Arm Trajec").getDoubleTopic("measurmentY").publish();
        setpointUpper = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Upper").publish();
        setpointLower = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Lower").publish();

        // addRequirements(m_robot);
    }

    @Override
    public void initialize() {
        m_lowerController.setIntegratorRange(1, 1);
        m_upperController.setIntegratorRange(1, 1);
    m_lowerController.setTolerance(m_config.tolerance);
    m_upperController.setTolerance(m_config.tolerance);
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
            m_kinematics.forward(m_robot.getMeasurement()),m_set, m_startAngle, m_endAngle);
           
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
        double desiredAcceleration = desiredState.accelerationMetersPerSecondSq;
        if (desiredState == m_trajectory.sample(10)) {
            desiredAcceleration = 0;
        }
        double theta = desiredState.poseMeters.getRotation().getRadians();
        double desiredXVel = desiredVecloity*Math.cos(theta);
        double desiredYVel = desiredVecloity*Math.cos(Math.PI/2-theta);
        double desiredXAccel = desiredAcceleration*Math.cos(theta);
        double desiredYAccel = desiredAcceleration*Math.cos(Math.PI/2-theta);
        Translation2d vel = new Translation2d(desiredXVel, desiredYVel);
        Translation2d accel = new Translation2d(desiredXAccel, desiredYAccel);
        vel = vel.plus(accel.times(m_robot.kA));
        System.out.println(accel);
        System.out.println(theta);
        System.out.println(desiredAcceleration);
        Translation2d XYreference = new Translation2d(desiredXPos, desiredYPos);
        ArmAngles thetaReference = m_kinematics.inverse(XYreference);
        ArmAngles inverseVel = m_kinematics.inverseVel(thetaReference, vel);
    double lowerControllerOutput = m_lowerController.calculate(measurement.th1, inverseVel.th1);
    double lowerFeedForward = thetaReference.th1/(Math.PI*2)*4;
    double u1 = lowerFeedForward;
    double upperControllerOutput = m_upperController.calculate(measurement.th2, inverseVel.th2);
    double upperFeedForward = thetaReference.th2/(Math.PI*2)*4;
    double u2 = upperFeedForward;
    m_robot.set(u1, u2);
    SmartDashboard.putNumber("Lower Encoder: ", measurement.th1);
    SmartDashboard.putNumber("Lower FF ", lowerFeedForward);
    SmartDashboard.putNumber("Lower Controller Output: ", lowerControllerOutput);
    SmartDashboard.putNumber("Upper FF ", upperFeedForward);
    SmartDashboard.putNumber("Upper Controller Output: ", upperControllerOutput);
    SmartDashboard.putNumber("Lower Ref: ", thetaReference.th1);
    SmartDashboard.putNumber("Upper Encoder: ", measurement.th2);
    SmartDashboard.putNumber("Upper Ref: ", thetaReference.th2);
    SmartDashboard.putNumber("Output Upper: ", u1);
    SmartDashboard.putNumber("Output Lower: ", u2);
        measurmentX.set(currentUpper);
        measurmentY.set(currentLower);
        setpointUpper.set(desiredYPos);
        setpointLower.set(desiredXPos);
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
}
