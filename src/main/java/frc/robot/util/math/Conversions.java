package frc.robot.util.math;

public final class Conversions {
	/**
	 * @param counts    Falcon Position Counts (Revolutions)
	 * @param gearRatio Gear Ratio between Falcon and Mechanism
	 * @return Degrees of Rotation of Mechanism
	 */
	public static double falconToDegrees(double positionCounts, double gearRatio) {
		return positionCounts * (360.0 / gearRatio);
	}

	/**
	 * @param degrees   Degrees of rotation of Mechanism
	 * @param gearRatio Gear Ratio between Falcon and Mechanism
	 * @return Falcon Position Counts (Revolutions)
	 */
	public static double degreesToFalcon(double degrees, double gearRatio) {
		return degrees / (360.0 / gearRatio);
	}

	/**
	 * @param velocitycounts Falcon Velocity Counts (Revolutions/Second)
	 * @param circumference  Circumference of Wheel (Meters)
	 * @param gearRatio      Gear Ratio between Falcon and Wheel
	 * @return Velocity (Meters/Second)
	 */
	public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
		return velocitycounts * circumference / gearRatio;
	}

	/**
	 * @param velocity      Velocity (Meters/Second)
	 * @param circumference Circumference of Wheel (Meters)
	 * @param gearRatio     Gear Ratio between Falcon and Wheel
	 * @return Falcon Velocity Counts (Revolutions/Second)
	 */
	public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
		return velocity / circumference * gearRatio;
	}

	/**
	 * @param positionCounts Falcon Position Counts (Revolutions)
	 * @param circumference  Circumference of Wheel (Meters)
	 * @param gearRatio      Gear Ratio between Falcon and Wheel
	 * @return Position (Meters)
	 */
	public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
		return positionCounts * circumference / gearRatio;
	}

	/**
	 * @param meters        Position (Meters)
	 * @param circumference Circumference of Wheel (Meters)
	 * @param gearRatio     Gear Ratio between Falcon and Wheel
	 * @return Falcon Position Counts (Revolutions)
	 */
	public static double metersToFalcon(double meters, double circumference, double gearRatio) {
		return meters / circumference * gearRatio;
	}
}