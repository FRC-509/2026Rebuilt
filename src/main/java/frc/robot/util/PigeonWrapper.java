package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class PigeonWrapper {
	private Pigeon2 device;
	private double yawOffset = 0.0d;
	private double bootRoll;
	private double bootPitch;

	public PigeonWrapper(int deviceId) {
		this.device = new Pigeon2(deviceId, Constants.kRio);
	}

	// TODO: Make sure that getValueAsDouble() returns the same as old getValue() did (IE -180 to 180)
	public void onEnable() {
		bootRoll = device.getRoll().getValueAsDouble();
	}

	public double getYaw() {
		return device.getYaw().getValueAsDouble() + yawOffset;
	}

	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(device.getYaw().getValueAsDouble() + yawOffset);
	}

	public void setYaw(double yaw) {
		yawOffset = yaw - device.getYaw().getValueAsDouble();
	}

	public double getPitch() {
		return device.getPitch().getValueAsDouble() - bootPitch;
	}

	public double getRoll() {
		return device.getRoll().getValueAsDouble() - bootRoll;
	}

	public double getAngularVelocityZWorld() {
		return device.getAngularVelocityZWorld().getValueAsDouble();
	}
}
