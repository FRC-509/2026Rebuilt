package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

public class AlignToHeading extends Command {
    
    private final SwerveDrive swerve;
	private final DoubleSupplier translationXSupplier;
	private final DoubleSupplier translationYSupplier;
    private final BooleanSupplier preciseMovementSupplier;

    private double heading;
    private PIDController controller = new PIDController(0.055, 0, 0);

	public AlignToHeading(SwerveDrive swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier preciseMovementSupplier, double heading) {
		this.swerve = swerve;
		this.translationXSupplier = xSupplier;
		this.translationYSupplier = ySupplier;
		this.preciseMovementSupplier = preciseMovementSupplier;
        this.heading = heading;

		addRequirements(swerve);
	}


    @Override
    public void initialize() {
        if (heading == 180 && swerve.getYaw().getDegrees() < 0) heading = -180;
    }

	@Override
	public void execute() {
		double preciseMovement = preciseMovementSupplier.getAsBoolean() ? Constants.Operator.kPrecisionMovementMultiplier: 1.0;
        
        Translation2d trans = new Translation2d(
			translationXSupplier.getAsDouble() * preciseMovement, 
			translationYSupplier.getAsDouble() * preciseMovement).times(Constants.Chassis.kMaxSpeed);

		swerve.drive(
            trans, 
            controller.calculate( 
                swerve.getYaw().getDegrees(), 
                heading),
            true, false);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(heading, swerve.getYaw().getDegrees(), 0.25);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setTargetHeading(swerve.getYaw().getDegrees());
        swerve.drive(new Translation2d(), 0, true, false);
    }
}