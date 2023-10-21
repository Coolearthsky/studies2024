package frc.robot.arm;

import java.util.List;

import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

public class ArmTrajectories {
    public static class Config {
        // Cone
        public ArmKinematics kinematics = new ArmKinematics(0.93, .92);
        public ArmAngles test = kinematics.inverse(new Translation2d(3, 3));
        public ArmAngles t0 = kinematics.inverse(new Translation2d(1, 1));
        public ArmAngles t1 = kinematics.inverse(new Translation2d(1.1, 1));
        public ArmAngles t2 = kinematics.inverse(new Translation2d(1.1, 1.1));
        public ArmAngles t3 = kinematics.inverse(new Translation2d(1, 1.1));

        // Cube
        public ArmAngles midGoalCube = new ArmAngles(0.089803, 1.681915);
        public ArmAngles lowGoalCube = new ArmAngles(-0.049849, 2.271662);
        public ArmAngles subCube = new ArmAngles(-0.341841, 1.361939);
        public ArmAngles subToCube = new ArmAngles(-0.341841, 1.361939);
        public ArmAngles highGoalCube = new ArmAngles(0.316365, 1.147321);

        public ArmAngles safeBack = new ArmAngles(-0.55, 1.97);
        public ArmAngles safeGoalCone = new ArmAngles(-0.639248, 1.838205);
        public ArmAngles safeGoalCube = new ArmAngles(-0.639248, 1.838205);
        public ArmAngles safeWaypoint = new ArmAngles(-0.394089, 1.226285);
    }

    private final Config m_config = new Config();
    private final TrajectoryConfig trajecConfig;

    public ArmTrajectories(TrajectoryConfig config) {
        trajecConfig = config;
    }

    public Trajectory makeTrajectory(ArmAngles start) {
        if (start == null) // unreachable
            return null;
        if (m_config.t0 != null && m_config.t1 != null && m_config.t2 != null && m_config.t3 != null) {
            System.out.println(start.th1 + " " +  start.th2);
            return onePoint(start, m_config.highGoalCube, 90);
        } else {
            System.out.println("ERROR");
            return null;
        }
    }

    /** from current location to an endpoint */
    public Trajectory onePoint(ArmAngles start, ArmAngles end, double degrees) {
        return withList(start, List.of(), end, degrees);
    }

    // /** from current location, through a waypoint, to an endpoint */
    private Trajectory twoPoint(ArmAngles start, ArmAngles mid, ArmAngles end,
    double degrees) {
    return withList(start, List.of(new Translation2d(mid.th2, mid.th1)), end,
    degrees);
    }

    private Trajectory fivePoint(ArmAngles start, ArmAngles mid1, ArmAngles mid2, ArmAngles mid3, ArmAngles mid4,
            ArmAngles end, double degrees) {
        List<Translation2d> list = List.of(new Translation2d(mid1.th2, mid1.th1), new Translation2d(mid2.th2, mid2.th1),
                new Translation2d(mid3.th2, mid3.th1), new Translation2d(mid4.th2, mid4.th1));
        if (list != null) {
            return withList(start, list, end, degrees);
        } else {
            System.out.println("ERROR");
            return null;
        }
    }

    private Trajectory withList(ArmAngles start, List<Translation2d> list, ArmAngles end, double degrees) {
        System.out.println("Start lower theta: " + start.th1 + " Start upper theta: " + start.th2);
        System.out.println("End lower theta: " + end.th1 + " End upper theta: " + end.th2);
        try {
            return TrajectoryGenerator.generateTrajectory(startPose(start, degrees), list, endPose(end, degrees),
                    trajecConfig);
        } catch (TrajectoryGenerationException e) {
            e.printStackTrace();
            return null;}
    }

    // note proximal is y
    private Pose2d startPose(ArmAngles start, double degrees) {
        return new Pose2d(start.th2, start.th1, Rotation2d.fromDegrees(degrees));
    }

    private Pose2d endPose(ArmAngles angles, double degrees) {
        return new Pose2d(angles.th2, angles.th1, Rotation2d.fromDegrees(degrees));
    }

}
