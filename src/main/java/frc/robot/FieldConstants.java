package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldConstants {
    public static class HubConstants {
        public static final Pose2d RED_CENTER_OF_HUB_POSE =
                new Pose2d(Inches.of(181.56 + (2 * 143.50)), Inches.of(158.32), Rotation2d.kZero);
        public static final Pose2d BLUE_CENTER_OF_HUB_POSE =
                new Pose2d(Inches.of(181.56), Inches.of(158.32), Rotation2d.kZero);
    }
}
