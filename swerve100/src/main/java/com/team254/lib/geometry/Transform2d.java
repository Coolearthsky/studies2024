package com.team254.lib.geometry;

import java.util.Objects;

public class Transform2d extends edu.wpi.first.math.geometry.Transform2d {

    public Transform2d(Pose2d initial, Pose2d last) {
        super(initial, last);

    }

    public Transform2d(Translation2d translation, Rotation2d rotation) {
        super(translation, rotation);

    }

    public Transform2d(edu.wpi.first.math.geometry.Translation2d translation,
            edu.wpi.first.math.geometry.Rotation2d rotation) {
        super(translation, rotation);
    }

    public Transform2d() {
    }

    public Transform2d inverse() {
        return new Transform2d(
                getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()),
                getRotation().unaryMinus());
    }

    /**
     * Checks equality between this Transform2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Transform2d) {
            return ((Transform2d) obj).getTranslation().equals(getTranslation())
                    && ((Transform2d) obj).getRotation().equals(getRotation());
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getTranslation(), getRotation());
    }
}