package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

@FunctionalInterface
public interface Translation2dSupplier {
    Translation2d getAsTranslation2d();
}