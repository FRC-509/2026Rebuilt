package frc.robot.util.math;

/**
 * Represents a simple linear interpolator.
 */
public class Interpolator {
	private double scale;
	private double setPoint;
	private double trailingPoint;
	private double position;

    /**
     * Constructs an Interpolator with the specified scale.
     * @param scale The scaling factor for the interpolation.
     */
	public Interpolator(double scale) {
		this.scale = scale;
		this.setPoint = 0.0d;
		this.trailingPoint = 0.0d;
		this.position = 0.0d;
	}

    /**
     * Sets the target set point for the interpolation.
     * @param value The new set point value.
     */
	public void setPoint(double value) {
		setPoint = value;
	}

    /**
     * Gets the current position obtained through interpolation.
     * @return The current position.
     */
	public double getPosition() {
		return position;
	}

    /**
     * Updates the interpolation based on the elapsed time.
     * @param deltaTimeSeconds The time elapsed in seconds since the last update.
     * @return The updated position after interpolation.
     */
	public double update(double deltaTimeSeconds) {
		position += scale * (trailingPoint - position) * deltaTimeSeconds;
		boolean withinDeadband = Math.abs(trailingPoint - position) <= 0.1 * scale;
		if (withinDeadband) {
			trailingPoint = setPoint;
		}

		return position;
	}
}