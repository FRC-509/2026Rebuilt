package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDriveCommand extends Command {
	private final SwerveDrive swerve;
	// How much the robot should move FORWARD (towards the OPPOSING ALLIANCE WALL if
	// field-relative)
	private final DoubleSupplier translationXSupplier;
	// How much the robot should move to the LEFT
	private final DoubleSupplier translationYSupplier;
	// How much the robot should rotate COUNTER-CLOCKWISE
	private final DoubleSupplier rotationSupplier;
	private final BooleanSupplier fieldRelativeSupplier;
	private final BooleanSupplier preciseMovementSupplier;
	private final BooleanSupplier preciseRotationSupplier;

	public DefaultDriveCommand(SwerveDrive swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier, BooleanSupplier preciseMovementSupplier, BooleanSupplier preciseRotationSupplier, BooleanSupplier fieldRelativeSupplier) {
		this.swerve = swerve;
		this.translationXSupplier = xSupplier;
		this.translationYSupplier = ySupplier;
		this.rotationSupplier = omegaSupplier;
		this.preciseMovementSupplier = preciseMovementSupplier;
		this.preciseRotationSupplier = preciseRotationSupplier;
		this.fieldRelativeSupplier = fieldRelativeSupplier;

		addRequirements(swerve);
	}

	public DefaultDriveCommand(SwerveDrive swerve, double xSpeed, double ySpeed,
			double omegaSpeed, boolean fieldRelative) {
		this.swerve = swerve;
		this.translationXSupplier = () -> xSpeed;
		this.translationYSupplier = () -> ySpeed;
		this.rotationSupplier = () -> omegaSpeed;
		this.preciseMovementSupplier = () -> false;
		this.preciseRotationSupplier = () -> false;
		this.fieldRelativeSupplier = () -> fieldRelative;

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		// evaluate created members for x,y,theta, run drive command
		double preciseMovement = preciseMovementSupplier.getAsBoolean() ? Constants.Operator.kPrecisionMovementMultiplier: 1.0;
		double preciseRotation = preciseRotationSupplier.getAsBoolean() ? Constants.Operator.kPrecisionRotationMultiplier : 1.0;
		Translation2d trans = new Translation2d(
			translationXSupplier.getAsDouble() * preciseMovement, 
			translationYSupplier.getAsDouble() * preciseMovement)
				.times(Constants.Chassis.kMaxSpeed);
		swerve.drive(trans, rotationSupplier.getAsDouble() * preciseRotation * Constants.Chassis.kMaxAngularVelocity,
				fieldRelativeSupplier.getAsBoolean(), false);
	}
}