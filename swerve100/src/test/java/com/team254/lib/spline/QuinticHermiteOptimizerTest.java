package com.team254.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import com.team254.lib.geometry.Pose2dState;
import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import com.team254.lib.util.Util;

public class QuinticHermiteOptimizerTest {
    private static double kEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        Pose2dState a = new Pose2dState(new Translation2dState(0, 100), Rotation2dState.fromDegrees(270));
        Pose2dState b = new Pose2dState(new Translation2dState(50, 0), Rotation2dState.fromDegrees(0));
        Pose2dState c = new Pose2dState(new Translation2dState(100, 100), Rotation2dState.fromDegrees(90));

        List<QuinticHermiteSpline> splines = new ArrayList<>();
        splines.add(new QuinticHermiteSpline(a, b));
        splines.add(new QuinticHermiteSpline(b, c));

        long startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines) < 0.014);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));

        Pose2dState d = new Pose2dState(new Translation2dState(0, 0), Rotation2dState.fromDegrees(90));
        Pose2dState e = new Pose2dState(new Translation2dState(0, 50), Rotation2dState.fromDegrees(0));
        Pose2dState f = new Pose2dState(new Translation2dState(100, 0), Rotation2dState.fromDegrees(90));
        Pose2dState g = new Pose2dState(new Translation2dState(100, 100), Rotation2dState.fromDegrees(0));

        List<QuinticHermiteSpline> splines1 = new ArrayList<>();
        splines1.add(new QuinticHermiteSpline(d, e));
        splines1.add(new QuinticHermiteSpline(e, f));
        splines1.add(new QuinticHermiteSpline(f, g));

        startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines1) < 0.16);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));


        Pose2dState h = new Pose2dState(new Translation2dState(0, 0), Rotation2dState.fromDegrees(0));
        Pose2dState i = new Pose2dState(new Translation2dState(50, 0), Rotation2dState.fromDegrees(0));
        Pose2dState j = new Pose2dState(new Translation2dState(100, 50), Rotation2dState.fromDegrees(45));
        Pose2dState k = new Pose2dState(new Translation2dState(150, 0), Rotation2dState.fromDegrees(270));
        Pose2dState l = new Pose2dState(new Translation2dState(150, -50), Rotation2dState.fromDegrees(270));

        List<QuinticHermiteSpline> splines2 = new ArrayList<>();
        splines2.add(new QuinticHermiteSpline(h, i));
        splines2.add(new QuinticHermiteSpline(i, j));
        splines2.add(new QuinticHermiteSpline(j, k));
        splines2.add(new QuinticHermiteSpline(k, l));

        startTime = System.currentTimeMillis();
        assertTrue(QuinticHermiteSpline.optimizeSpline(splines2) < 0.05);
        assertEquals(splines2.get(0).getCurvature(1.0), 0.0, kEpsilon);
        assertEquals(splines2.get(2).getCurvature(1.0), 0.0, kEpsilon);
        System.out.println("Optimization time (ms): " + (System.currentTimeMillis() - startTime));
    }
}
