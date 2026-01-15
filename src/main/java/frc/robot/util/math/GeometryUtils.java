package frc.robot.util.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class GeometryUtils {
	public static Translation2d rotatePointAboutPoint2D(Translation2d point, Translation2d origin,
			Rotation2d rotation) {
		Translation2d originToPoint = point.minus(origin);
		Translation2d rotatedOriginToPoint = originToPoint.rotateBy(rotation);
		Translation2d rotatedPoint = rotatedOriginToPoint.plus(origin);
		return rotatedPoint;
	}

	public static Translation3d rotatePointAboutPoint3D(Translation3d point, Translation3d origin,
			Rotation3d rotation) {
		Translation3d originToPoint = point
				.minus(origin);
		Translation3d rotatedOriginToPoint = originToPoint.rotateBy(rotation);
		Translation3d rotatedPoint = rotatedOriginToPoint.plus(origin);
		return rotatedPoint;
	}

	public static Pose2d flipFieldPose(Pose2d pose) {
    return new Pose2d(
        new Translation2d(Constants.Field.kFullFieldLength - pose.getTranslation().getX(), pose.getTranslation().getY()),
		new Rotation2d(Math.PI).minus(pose.getRotation()));
 	}
}
