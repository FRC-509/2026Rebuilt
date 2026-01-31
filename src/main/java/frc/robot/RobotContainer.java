// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToHeading;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.HopperDefaultCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Vortex;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;
import frc.robot.util.controllers.ThrustmasterJoystick;

public class RobotContainer {
	
	private final PigeonWrapper pigeon = new PigeonWrapper(0);

	private final ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private final ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private final CommandXboxController operatorController = new CommandXboxController(2);
	
	private final SwerveDrive swerve;
	private final Hopper hopper;

	private final Vortex vortex;

	private SendableChooser<Command> chooser = new SendableChooser<Command>();

    public RobotContainer() {
		this.swerve = new SwerveDrive(pigeon);
		this.hopper = new Hopper();

		this.vortex = new Vortex(swerve, null);

		configureBindings();
		addAutonomousRoutines();
	}

    private void configureBindings() {
		
		swerve.setDefaultCommand(new DefaultDriveCommand(swerve,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX()),
				() -> driverLeft.getTrigger(),
				() -> driverRight.getTrigger(),
				() -> true));


		(new Trigger(() -> driverRight.getPOV(0) == 0)).onTrue(
			new AlignToHeading(
				swerve, 
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> driverLeft.getTrigger(),
				0));

		(new Trigger(() -> driverRight.getPOV(0) == 90)).onTrue(
			new AlignToHeading(
				swerve, 
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> driverLeft.getTrigger(),
				-90));

		(new Trigger(() -> driverRight.getPOV(0) == 270)).onTrue(
			new AlignToHeading(
				swerve, 
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> driverLeft.getTrigger(),
				90));

		(new Trigger(() -> driverRight.getPOV(0) == 180)).onTrue(
			new AlignToHeading(
				swerve, 
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> driverLeft.getTrigger(),
				180));


		hopper.setDefaultCommand(new HopperDefaultCommand(hopper,
			() -> driverRight.getTrigger(),
			() -> operatorController.getRightTriggerAxis() > 0.5,
			() -> operatorController.b().getAsBoolean(),
			() -> driverLeft.getTrigger()));
	}

	private void addAutonomousRoutines() {
		chooser.addOption("\"Go AFK\" (Null)", new InstantCommand());
		SmartDashboard.putData("Auto Mode", chooser);

		if (RobotBase.isSimulation()) {
			SmartDashboard.putData("Reset Swerve", Commands.runOnce(swerve::resetSimState, swerve));
		}
	}
	
    public Command getAutonomousCommand() {
      	return Commands.print("No autonomous command configured");
    }

	public void robotPeriodic() {
		vortex.updatePositionEstimate();
	}

	private static double nonInvSquare(double axis) {
		double deadbanded = MathUtil.applyDeadband(axis, Constants.Operator.kStickDeadband);
		double squared = Math.abs(deadbanded) * deadbanded;
		return squared;
	}
  
}
