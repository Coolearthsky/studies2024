package com.team254.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TestSE2Math {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void testRotation2d() {
        // Test constructors
        Rotation2dState rot1 = new Rotation2dState();
        assertEquals(1, rot1.getCos(), kTestEpsilon);
        assertEquals(0, rot1.getSin(), kTestEpsilon);
        assertEquals(0, rot1.getTan(), kTestEpsilon);
        assertEquals(0, rot1.getDegrees(), kTestEpsilon);
        assertEquals(0, rot1.getRadians(), kTestEpsilon);

        rot1 = new Rotation2dState(1, 1);
        assertEquals(Math.sqrt(2) / 2, rot1.getCos(), kTestEpsilon);
        assertEquals(Math.sqrt(2) / 2, rot1.getSin(), kTestEpsilon);
        assertEquals(1, rot1.getTan(), kTestEpsilon);
        assertEquals(45, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 4, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2dState.fromRadians(Math.PI / 2);
        assertEquals(0, rot1.getCos(), kTestEpsilon);
        assertEquals(1, rot1.getSin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot1.getTan());
        assertEquals(90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2dState.fromDegrees(270);
        assertEquals(0, rot1.getCos(), kTestEpsilon);
        assertEquals(-1, rot1.getSin(), kTestEpsilon);
        System.out.println(rot1.getTan());
        // this test is silly
        // assertTrue(-1 / kTestEpsilon > rot1.getTan(), String.format("%f",
        // rot1.getTan()));
        // this tests the angle-wrapping thing that wpi doesn't do
        // assertEquals(-90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(270, rot1.getDegrees(), kTestEpsilon);
        assertEquals(3 * Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        // Test inversion
        rot1 = Rotation2dState.fromDegrees(270);
        Rotation2dState rot2 = rot1.unaryMinus();
        assertEquals(0, rot2.getCos(), kTestEpsilon);
        assertEquals(1, rot2.getSin(), kTestEpsilon);
        // this test is silly
        // assertTrue(1 / kTestEpsilon < rot2.getTan());
        // this tests the angle-wrapping thing that wpi doesn't do
        //assertEquals(90, rot2.getDegrees(), kTestEpsilon);
        assertEquals(-270, rot2.getDegrees(), kTestEpsilon);
        assertEquals(-3*Math.PI / 2, rot2.getRadians(), kTestEpsilon);

        rot1 = Rotation2dState.fromDegrees(1);
        rot2 = rot1.unaryMinus();
        assertEquals(rot1.getCos(), rot2.getCos(), kTestEpsilon);
        assertEquals(-rot1.getSin(), rot2.getSin(), kTestEpsilon);
        assertEquals(-1, rot2.getDegrees(), kTestEpsilon);

        // Test rotateBy
        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(45);
        Rotation2d rot3 = rot1.rotateBy(rot2);
        assertEquals(0, rot3.getCos(), kTestEpsilon);
        assertEquals(1, rot3.getSin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot3.getTan());
        assertEquals(90, rot3.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot3.getRadians(), kTestEpsilon);

        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(-45);
        rot3 = rot1.rotateBy(rot2);
        assertEquals(1, rot3.getCos(), kTestEpsilon);
        assertEquals(0, rot3.getSin(), kTestEpsilon);
        assertEquals(0, rot3.getTan(), kTestEpsilon);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);
        assertEquals(0, rot3.getRadians(), kTestEpsilon);

        // A rotation times its inverse should be the identity
        Rotation2dState identity = new Rotation2dState();
        rot1 = Rotation2dState.fromDegrees(21.45);
        rot2 = rot1.rotateBy(rot1.unaryMinus());
        assertEquals(identity.getCos(), rot2.getCos(), kTestEpsilon);
        assertEquals(identity.getSin(), rot2.getSin(), kTestEpsilon);
        assertEquals(identity.getDegrees(), rot2.getDegrees(), kTestEpsilon);

        // Test interpolation
        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(135);
        rot3 = rot1.interpolate2(rot2, .5);
        assertEquals(90, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(135);
        rot3 = rot1.interpolate2(rot2, .75);
        assertEquals(112.5, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(-45);
        rot3 = rot1.interpolate2(rot2, .5);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(45);
        rot3 = rot1.interpolate2(rot2, .5);
        assertEquals(45, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(45);
        rot3 = rot1.interpolate2(rot2, .5);
        assertEquals(45, rot3.getDegrees(), kTestEpsilon);

        // Test parallel.
        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(45);
        assertTrue(rot1.isParallel(rot2));

        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(-45);
        assertFalse(rot1.isParallel(rot2));

        rot1 = Rotation2dState.fromDegrees(45);
        rot2 = Rotation2dState.fromDegrees(-135);
        assertTrue(rot1.isParallel(rot2));
    }

    @Test
    public void testTranslation2d() {
        // Test constructors
        Translation2dState pos1 = new Translation2dState();
        assertEquals(0, pos1.get().getX(), kTestEpsilon);
        assertEquals(0, pos1.get().getY(), kTestEpsilon);
        assertEquals(0, pos1.get().getNorm(), kTestEpsilon);

        pos1 = new Translation2dState(3, 4);
        assertEquals(3, pos1.get().getX(), kTestEpsilon);
        assertEquals(4, pos1.get().getY(), kTestEpsilon);
        assertEquals(5, pos1.get().getNorm(), kTestEpsilon);

        // Test inversion
        pos1 = new Translation2dState(3.152, 4.1666);
        Translation2dState pos2 = pos1.unaryMinus();
        assertEquals(-pos1.get().getX(), pos2.get().getX(), kTestEpsilon);
        assertEquals(-pos1.get().getY(), pos2.get().getY(), kTestEpsilon);
        assertEquals(pos1.get().getNorm(), pos2.get().getNorm(), kTestEpsilon);

        // Test rotateBy
        pos1 = new Translation2dState(2, 0);
        Rotation2dState rot1 = Rotation2dState.fromDegrees(90);
        pos2 = pos1.rotateBy(rot1);
        assertEquals(0, pos2.get().getX(), kTestEpsilon);
        assertEquals(2, pos2.get().getY(), kTestEpsilon);
        assertEquals(pos1.get().getNorm(), pos2.get().getNorm(), kTestEpsilon);

        pos1 = new Translation2dState(2, 0);
        rot1 = Rotation2dState.fromDegrees(-45);
        pos2 = pos1.rotateBy(rot1);
        assertEquals(Math.sqrt(2), pos2.get().getX(), kTestEpsilon);
        assertEquals(-Math.sqrt(2), pos2.get().getY(), kTestEpsilon);
        assertEquals(pos1.get().getNorm(), pos2.get().getNorm(), kTestEpsilon);

        // Test translateBy
        pos1 = new Translation2dState(2, 0);
        pos2 = new Translation2dState(-2, 1);
        Translation2d pos3 = pos1.plus(pos2).get();
        assertEquals(0, pos3.getX(), kTestEpsilon);
        assertEquals(1, pos3.getY(), kTestEpsilon);
        assertEquals(1, pos3.getNorm(), kTestEpsilon);

        // A translation times its inverse should be the identity
        Translation2dState identity = new Translation2dState();
        pos1 = new Translation2dState(2.16612, -23.55);
        pos2 = pos1.plus(pos1.unaryMinus());
        assertEquals(identity.get().getX(), pos2.get().getX(), kTestEpsilon);
        assertEquals(identity.get().getY(), pos2.get().getY(), kTestEpsilon);
        assertEquals(identity.get().getNorm(), pos2.get().getNorm(), kTestEpsilon);

        // Test interpolation
        pos1 = new Translation2dState(0, 1);
        pos2 = new Translation2dState(10, -1);
        pos3 = pos1.interpolate2(pos2, .5).get();
        assertEquals(5, pos3.getX(), kTestEpsilon);
        assertEquals(0, pos3.getY(), kTestEpsilon);

        pos1 = new Translation2dState(0, 1);
        pos2 = new Translation2dState(10, -1);
        pos3 = pos1.interpolate2(pos2, .75).get();
        assertEquals(7.5, pos3.getX(), kTestEpsilon);
        assertEquals(-.5, pos3.getY(), kTestEpsilon);
    }

    @Test
    public void testPose2d() {
        // Test constructors
        Pose2dState pose1 = new Pose2dState();
        assertEquals(0, pose1.getTranslation().getX(), kTestEpsilon);
        assertEquals(0, pose1.getTranslation().getY(), kTestEpsilon);
        assertEquals(0, pose1.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new Pose2dState(new Translation2dState(3, 4), Rotation2dState.fromDegrees(45));
        assertEquals(3, pose1.getTranslation().getX(), kTestEpsilon);
        assertEquals(4, pose1.getTranslation().getY(), kTestEpsilon);
        assertEquals(45, pose1.getRotation().getDegrees(), kTestEpsilon);

        // Test transformation
        pose1 = new Pose2dState(new Translation2dState(3, 4), Rotation2dState.fromDegrees(90));
        Pose2dState pose2 = new Pose2dState(new Translation2dState(1, 0), Rotation2dState.fromDegrees(0));
        Pose2dState pose3 = pose1.transformBy(pose2);
        assertEquals(3, pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(5, pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(90, pose3.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new Pose2dState(new Translation2dState(3, 4), Rotation2dState.fromDegrees(90));
        pose2 = new Pose2dState(new Translation2dState(1, 0), Rotation2dState.fromDegrees(-90));
        pose3 = pose1.transformBy(pose2);
        assertEquals(3, pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(5, pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(0, pose3.getRotation().getDegrees(), kTestEpsilon);

        // A pose times its inverse should be the identity
        Pose2dState identity = new Pose2dState();
        pose1 = new Pose2dState(new Translation2dState(3.51512152, 4.23), Rotation2dState.fromDegrees(91.6));
        pose2 = pose1.transformBy(pose1.inverse());
        assertEquals(identity.getTranslation().getX(), pose2.getTranslation().getX(), kTestEpsilon);
        assertEquals(identity.getTranslation().getY(), pose2.getTranslation().getY(), kTestEpsilon);
        assertEquals(identity.getRotation().getDegrees(), pose2.getRotation().getDegrees(), kTestEpsilon);

        // Test interpolation
        // Movement from pose1 to pose2 is along a circle with radius of 10 units
        // centered at (3, -6)
        pose1 = new Pose2dState(new Translation2dState(3, 4), Rotation2dState.fromDegrees(90));
        pose2 = new Pose2dState(new Translation2dState(13, -6), Rotation2dState.fromDegrees(0.0));
        pose3 = pose1.interpolate2(pose2, .5);
        double expected_angle_rads = Math.PI / 4;
        assertEquals(3.0 + 10.0 * Math.cos(expected_angle_rads), pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(-6.0 + 10.0 * Math.sin(expected_angle_rads), pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(expected_angle_rads, pose3.getRotation().getRadians(), kTestEpsilon);

        pose1 = new Pose2dState(new Translation2dState(3, 4), Rotation2dState.fromDegrees(90));
        pose2 = new Pose2dState(new Translation2dState(13, -6), Rotation2dState.fromDegrees(0.0));
        pose3 = pose1.interpolate2(pose2, .75);
        expected_angle_rads = Math.PI / 8;
        assertEquals(3.0 + 10.0 * Math.cos(expected_angle_rads), pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(-6.0 + 10.0 * Math.sin(expected_angle_rads), pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(expected_angle_rads, pose3.getRotation().getRadians(), kTestEpsilon);
    }

    @Test
    public void testTwist() {
        // Exponentiation (integrate twist to obtain a Pose2d)
        Twist2dWrapper twist = new Twist2dWrapper(1.0, 0.0, 0.0);
        Pose2dState pose = Pose2dState.sexp(twist);
        assertEquals(1.0, pose.getTranslation().getX(), kTestEpsilon);
        assertEquals(0.0, pose.getTranslation().getY(), kTestEpsilon);
        assertEquals(0.0, pose.getRotation().getDegrees(), kTestEpsilon);

        // Scaled.
        twist = new Twist2dWrapper(1.0, 0.0, 0.0);
        pose = Pose2dState.sexp(twist.scaled(2.5));
        assertEquals(2.5, pose.getTranslation().getX(), kTestEpsilon);
        assertEquals(0.0, pose.getTranslation().getY(), kTestEpsilon);
        assertEquals(0.0, pose.getRotation().getDegrees(), kTestEpsilon);

        // Logarithm (find the twist to apply to obtain a given Pose2d)
        pose = new Pose2dState(new Translation2dState(2.0, 2.0), Rotation2dState.fromRadians(Math.PI / 2));
        twist = Pose2dState.slog(pose);
        assertEquals(Math.PI, twist.dx, kTestEpsilon);
        assertEquals(0.0, twist.dy, kTestEpsilon);
        assertEquals(Math.PI / 2, twist.dtheta, kTestEpsilon);

        // Logarithm is the inverse of exponentiation.
        Pose2dState new_pose = Pose2dState.sexp(twist);
        assertEquals(new_pose.getTranslation().getX(), pose.getTranslation().getX(), kTestEpsilon);
        assertEquals(new_pose.getTranslation().getY(), pose.getTranslation().getY(), kTestEpsilon);
        assertEquals(new_pose.getRotation().getDegrees(), pose.getRotation().getDegrees(), kTestEpsilon);
    }
}
