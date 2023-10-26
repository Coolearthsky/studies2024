package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.armMotion.ArmKinematics;

public class Sequence extends SequentialCommandGroup {
    private final Robot m_robot;
    private final ArmKinematics m_kinematics;
    public Sequence(Robot robot, ArmKinematics kinematics) {
        m_robot = robot;
        m_kinematics = kinematics;
        addCommands(new ArmTrajectory(m_robot, new Translation2d(.6, .6),m_kinematics), 
        new ArmTrajectory(m_robot, new Translation2d(1, .6),m_kinematics),
        new ArmTrajectory(m_robot, new Translation2d(1, 1),m_kinematics),
        new ArmTrajectory(m_robot, new Translation2d(.6, 1),m_kinematics),
        new ArmTrajectory(m_robot, new Translation2d(.6, .6),m_kinematics)
        );
    }
}
