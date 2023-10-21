package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.org.team100.planning.OperationExecutor;
import frc.robot.org.team100.planning.SvgReader;
import frc.robot.org.team100.plotter.Operation;
import frc.robot.org.team100.plotter.SVGToPlotOperations;

public class SvgTest {

    private static InputStream stream(String filename) throws IOException {
        Path deployPath = Filesystem.getDeployDirectory().toPath();
        Path path = deployPath.resolve(filename);
        return Files.newInputStream(path);
    }

    @Test
    void testSvg1() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 1;
        double yScale = -1;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("center_mark.svg"), ops);

        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op : ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(1000);
        System.out.println("done");
    }

    @Test
    void testSvg2() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 0.00025;
        double yScale = -0.00025;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("hat.svg"), ops);

        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op : ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(10000);
        System.out.println("done");
    }

    @Test
    void testSvg3() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 1;
        double yScale = -1;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("circle.svg"), ops);

        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op : ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(10000);

        // so this circle should actually be a circle
        List<Operation> oplist = ops.getOperations();
        Translation2d c = new Translation2d(12, -12);
        for (int i = 1; i < 5; ++i) {
            Trajectory trajectory = oplist.get(i).getTrajectory();

            for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += 0.1) {
                Trajectory.State state = trajectory.sample(t);
                Pose2d pose = state.poseMeters;
                Translation2d tr = pose.getTranslation();
                double norm = tr.minus(c).getNorm();
                System.out.printf("tr %s d %f\n", tr, norm);
                // so this is within 1/3000th of a circle 
                // which seems good enough
                assertEquals(9, norm, 0.003);
            }
        }

        System.out.println("done");
    }

    @Test
    void testSvg4() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 1;
        double yScale = -1;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("penrose_triangle.svg"), ops);

        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op : ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(1000);
        System.out.println("done");
    }

    @Test
    void testSvg5() throws InterruptedException, IOException {
        System.out.println("start");
        double xScale = 1;
        double yScale = -1;

        SVGToPlotOperations ops = new SVGToPlotOperations(xScale, yScale);
        SvgReader reader = new SvgReader(stream("subpop.svg"), ops);
        reader.run();

        // this would be in the command initialize() method
        OperationExecutor exec = new OperationExecutor();

        // this would be in the command execute() method.

        for (Operation op : ops.getOperations()) {
            exec.executeTrajectory(op.isPenDown(), op.getTrajectory());
        }

        // let the user look at the picture.
        Thread.sleep(1000);
        System.out.println("done");
    }
}
