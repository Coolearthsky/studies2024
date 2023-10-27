package com.team254.lib.util;

public class Units {
    public static final double kInchesToMeters = 0.0254;

    public static double inches_to_meters(double inches) {
        return inches * kInchesToMeters;
    }

    public static double meters_to_inches(double meters) {
        return meters / kInchesToMeters;
    }


}
