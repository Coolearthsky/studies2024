package com.team254.lib.physics;

import com.team254.lib.geometry.Rotation2dState;
import com.team254.lib.geometry.Translation2dState;
import java.text.DecimalFormat;

import org.team100.lib.geometry.GeometryUtil;

public class SwerveDrive {
    // All units must be SI!

    // Self-explanatory.  Measure by rolling the robot a known distance and counting encoder ticks.
    protected final double wheel_radius_;  // m

    // "Effective" kinematic wheelbase radius.  Might be larger than theoretical to compensate for skid steer.  Measure
    // by turning the robot in place several times and figuring out what the equivalent wheelbase radius is.
    protected final double effective_wheelbase_radius_;  // m

    public SwerveDrive(final double wheel_radius,
                       final double effective_wheelbase_radius) {
        wheel_radius_ = wheel_radius;
        effective_wheelbase_radius_ = effective_wheelbase_radius;
    }

    // Can refer to velocity or acceleration depending on context.
    public static class ChassisState {
        public Translation2dState movement;
        public Rotation2dState heading;

        public ChassisState(Translation2dState movement, Rotation2dState heading) {
            this.heading = heading;
            this.movement = movement;
        }

        public ChassisState(Translation2dState movement) {
            this.movement = movement;
            this.heading = GeometryUtil.kRotationIdentity;
        }


        public ChassisState() {
            this.movement = GeometryUtil.kTranslation2dIdentity;
            this.heading = GeometryUtil.kRotationIdentity;
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(movement.get().getNorm())/* + ", " + fmt.format(heading.getRadians())*/;
        }
    }
}