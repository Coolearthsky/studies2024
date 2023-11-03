package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmSubsystem;

public class Sequence extends SequentialCommandGroup {
    private final ArmSubsystem m_robot;
    public Sequence(ArmSubsystem robot) {
        m_robot = robot;
        addCommands(new ArmTrajectory(m_robot, new Translation2d(.6, .6), 90, 90), 
        new ArmTrajectory(m_robot, new Translation2d(1, .6), 0 ,0),
        new ArmTrajectory(m_robot, new Translation2d(1, 1),90,90),
        new ArmTrajectory(m_robot, new Translation2d(.6, 1),180,180),
        new ArmTrajectory(m_robot, new Translation2d(.6, .6),270 ,270)
        );
    }
}
