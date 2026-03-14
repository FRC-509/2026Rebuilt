// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import java.util.stream.Stream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.AimTarget;
import frc.robot.subsystems.Vortex;
import frc.robot.commands.ChoreoAuto;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.HopperDefaultCommand;
import frc.robot.commands.RightSprintAndLever;
import frc.robot.commands.ShootPreloadAuto;
import frc.robot.subsystems.GameManager;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PigeonWrapper;
import frc.robot.util.Translation2dSupplier;
import frc.robot.util.controllers.ThrustmasterJoystick;
import frc.robot.util.controllers.ThrustmasterJoystick.StickButton;

public class RobotContainer {
	
	private final PigeonWrapper pigeon = new PigeonWrapper(0);

	private final ThrustmasterJoystick driverLeft = new ThrustmasterJoystick(0);
	private final ThrustmasterJoystick driverRight = new ThrustmasterJoystick(1);
	private final CommandXboxController operatorController = new CommandXboxController(2);
	
	private final SwerveDrive swerve;
	private final Turret leftTurret;
	private final Turret rightTurret;
	private final Hopper hopper;
	private final Vortex vortex;
	private GameManager gameManager;
	private final NetworkTable elasticTable;
	private final ShuffleboardTab elasticTab;

	private SendableChooser<Command> chooser = new SendableChooser<Command>();

    public RobotContainer() {
		this.swerve = new SwerveDrive(pigeon);
		this.gameManager = new GameManager();
		this.hopper = new Hopper();
		this.vortex = new Vortex(swerve, new Pose2d(), () -> 0);
		this.elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
		this.elasticTab = Shuffleboard.getTab("Elastic");

		this.leftTurret = new Turret(
			Constants.Turret.kLeftTurretConfiguration,
			new Translation2dSupplier() { public Translation2d getAsTranslation2d() { return vortex.getEstimatedAlliancePosition(); } },
			new Translation2dSupplier() {
				public Translation2d getAsTranslation2d() {
					ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), swerve.getYaw());
					return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
				}
			},
			() -> swerve.getYaw().getRadians(),
			hopper::isIndexing);
			
			
		this.rightTurret = new Turret(
			Constants.Turret.kRightTurretConfiguration,
			new Translation2dSupplier() { public Translation2d getAsTranslation2d() { return vortex.getEstimatedAlliancePosition(); } },
			new Translation2dSupplier() {
				public Translation2d getAsTranslation2d() {
					ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), swerve.getYaw());
					return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
				}
			},
			() -> swerve.getYaw().getRadians(),
			hopper::isIndexing);

		configureBindings();
		addAutonomousRoutines();
		configureElastic();
	}

    private void configureBindings() {
		
		swerve.setDefaultCommand(new DefaultDriveCommand(swerve,
				() -> nonInvSquare(-driverLeft.getY()),
				() -> nonInvSquare(-driverLeft.getX()),
				() -> nonInvSquare(-driverRight.getX()),
				() -> driverLeft.getTrigger(),
				() -> driverRight.getTrigger(),
				() -> true));

		driverLeft.isPressedBind(StickButton.Left, Commands.runOnce(() -> {
			pigeon.setYaw(0);
			swerve.setTargetHeading(0);
		}, swerve));

		// (new Trigger(() -> driverRight.getPOV(0) == 0)).onTrue(
		// 	new AlignToHeading(
		// 		swerve, 
		// 		() -> nonInvSquare(-driverLeft.getY()),
		// 		() -> nonInvSquare(-driverLeft.getX()),
		// 		() -> driverLeft.getTrigger(),
		// 		0));

		// (new Trigger(() -> driverRight.getPOV(0) == 90)).onTrue(
		// 	new AlignToHeading(
		// 		swerve, 
		// 		() -> nonInvSquare(-driverLeft.getY()),
		// 		() -> nonInvSquare(-driverLeft.getX()),
		// 		() -> driverLeft.getTrigger(),
		// 		-90));

		// (new Trigger(() -> driverRight.getPOV(0) == 270)).onTrue(
		// 	new AlignToHeading(
		// 		swerve, 
		// 		() -> nonInvSquare(-driverLeft.getY()),
		// 		() -> nonInvSquare(-driverLeft.getX()),
		// 		() -> driverLeft.getTrigger(),
		// 		90));

		// (new Trigger(() -> driverRight.getPOV(0) == 180)).onTrue(
		// 	new AlignToHeading(
		// 		swerve, 
		// 		() -> nonInvSquare(-driverLeft.getY()),
		// 		() -> nonInvSquare(-driverLeft.getX()),
		// 		() -> driverLeft.getTrigger(),
		// 		180));


		hopper.setDefaultCommand(new HopperDefaultCommand(hopper,
			() -> driverRight.getTrigger(),
			() -> driverRight.isPressed(StickButton.Bottom),
			() -> operatorController.getRightTriggerAxis() > 0.7,
			() -> operatorController.b().getAsBoolean(),
			() -> Math.abs(operatorController.getLeftTriggerAxis()) > 0.7 && gameManager.shouldPrefire(Constants.Hopper.kPrefireLeadTimeSeconds) && LimelightHelpers.getTV(Constants.Vortex.kFrontLimelightName),
			() -> Math.abs(operatorController.getRightTriggerAxis()) > 0.7,
			() -> Math.abs(operatorController.getRightTriggerAxis()) > 0.7,
			() -> true,
			() -> true));


		// force feed override
		// (new Trigger(() -> operatorController.povLeft().getAsBoolean()))
		// 	.onTrue(Commands.runOnce(
		// 		() -> {
		// 			leftTurret.setOverrideAimTarget(true, AimTarget.OPPOSING_ALLIANCE_FEED_LEFT);
		// 			rightTurret.setOverrideAimTarget(true, AimTarget.OPPOSING_ALLIANCE_FEED_LEFT);
		// 		}, leftTurret, rightTurret))
		// 	.onFalse(Commands.runOnce(
		// 	() -> {
		// 		leftTurret.setOverrideAimTarget(false, AimTarget.NONE);
		// 		rightTurret.setOverrideAimTarget(false, AimTarget.NONE);
		// 	}, leftTurret, rightTurret));
		
		// (new Trigger(() -> operatorController.povRight().getAsBoolean()))
		// 	.onTrue(Commands.runOnce(
		// 		() -> {
		// 			leftTurret.setOverrideAimTarget(true, AimTarget.OPPOSING_ALLIANCE_FEED_RIGHT);
		// 			rightTurret.setOverrideAimTarget(true, AimTarget.OPPOSING_ALLIANCE_FEED_RIGHT);
		// 		}, leftTurret, rightTurret))
		// 	.onFalse(Commands.runOnce(
		// 	() -> {
		// 		leftTurret.setOverrideAimTarget(false, AimTarget.NONE);
		// 		rightTurret.setOverrideAimTarget(false, AimTarget.NONE);
		// 	}, leftTurret, rightTurret));
		
		// (new Trigger(() -> operatorController.povDown().getAsBoolean())) // force split feed
		// 	.onTrue(Commands.runOnce(
		// 		() -> {
		// 			leftTurret.setOverrideAimTarget(true, AimTarget.OPPOSING_ALLIANCE_FEED_RIGHT);
		// 			rightTurret.setOverrideAimTarget(true, AimTarget.OPPOSING_ALLIANCE_FEED_LEFT);
		// 		}, leftTurret, rightTurret))
		// 	.onFalse(Commands.runOnce(
		// 	() -> {
		// 		leftTurret.setOverrideAimTarget(false, AimTarget.NONE);
		// 		rightTurret.setOverrideAimTarget(false, AimTarget.NONE);
		// 	}, leftTurret, rightTurret));

		operatorController.y().onTrue(Commands.runOnce(gameManager::confirmAutoWin, gameManager));
		operatorController.x().onTrue(Commands.runOnce(gameManager::clearAutoWin, gameManager));
		operatorController.a().onTrue(Commands.runOnce(() -> {
			leftTurret.zeroPosition();
			rightTurret.zeroPosition();
		}));
	}

	private void addAutonomousRoutines() {
		chooser.addOption("\"Go AFK\" (Null)", new InstantCommand());
		chooser.addOption("Shoot Preload", ShootPreloadAuto.create(hopper));
		chooser.addOption("RightSprintAndLever", new RightSprintAndLever(swerve, pigeon, hopper, leftTurret, rightTurret));
		Path choreoDirectory = Filesystem.getDeployDirectory().toPath().resolve("choreo");
		// try (Stream<Path> choreoFiles = Files.list(choreoDirectory)) {
		// 	choreoFiles
		// 		.filter(path -> path.getFileName().toString().endsWith(".traj"))
		// 		forEach(path -> {
		// 			String fileName = path.getFileName().toString();
		// 			String trajectoryName = fileName.substring(0, fileName.length() - ".traj".length());
		// 			if (!"NewPath".equals(trajectoryName)) {
		// 				chooser.addOption("Choreo: " + trajectoryName, new ChoreoAuto(trajectoryName, swerve, pigeon));
		// 			}
		// 		});
		// } catch (IOException ignored) {
		// }
	}

	private void configureElastic() {
		elasticTab.add("Robot Field", vortex.getField())
			.withWidget(BuiltInWidgets.kField)
			.withPosition(0, 0)
			.withSize(6, 4);
		elasticTab.add("Auto Mode", chooser)
			.withWidget(BuiltInWidgets.kComboBoxChooser)
			.withPosition(6, 0)
			.withSize(3, 1);
		elasticTab.addDouble("Match Timer", gameManager::getMatchTimeSeconds).
			withWidget(BuiltInWidgets.kDial);
		elasticTab.addString("Match Phase", () -> gameManager.getCurrentPhase().name())
			.withWidget(BuiltInWidgets.kTextView)
			.withPosition(8, 1)
			.withSize(2, 1);
		elasticTab.addBoolean("Hub is Active", gameManager::isHubActive)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withPosition(8, 2)
			.withSize(1, 1);
		elasticTab.addDouble("Time Until Shift", gameManager::getTimeToNextShift)
			.withWidget(BuiltInWidgets.kNumberBar)
			.withProperties(Map.of("Min", -1, "Max", 30))
			.withPosition(9, 2)
			.withSize(1, 1);
		elasticTab.addDouble("Swerve Yaw", () -> swerve.getYaw().getDegrees())
			.withWidget(BuiltInWidgets.kDial)
			.withPosition(6, 2)
			.withSize(2, 2);
		elasticTab.addBoolean("Left Can Aim", leftTurret::canAim)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withPosition(8, 3)
			.withSize(1, 1);
		elasticTab.addBoolean("Right Can Aim", rightTurret::canAim)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.withPosition(9, 3)
			.withSize(1, 1);

		if (RobotBase.isSimulation()) {
			elasticTab.add("Reset Swerve", Commands.runOnce(swerve::resetSimState, swerve))
				.withPosition(9, 0)
				.withSize(2, 1);
		}
	}

	public Command getAutonomousCommand() {
      	Command selected = chooser.getSelected();
		return selected != null ? selected : Commands.print("No autonomous command configured");
	}

	public void robotPeriodic() {
		vortex.pollJetsons();
		vortex.getField().getObject("left_turret").setPose(leftTurret.getTurretFieldPose());
		vortex.getField().getObject("right_turret").setPose(rightTurret.getTurretFieldPose());

		elasticTable.getEntry("MatchTimer").setString(gameManager.getFormattedMatchTime());
		elasticTable.getEntry("MatchTimeSeconds").setDouble(gameManager.getMatchTimeSeconds());
		elasticTable.getEntry("MatchPhase").setString(gameManager.getCurrentPhase().name());
		elasticTable.getEntry("HubActive").setBoolean(gameManager.isHubActive());
		elasticTable.getEntry("TimeToNextShiftSeconds").setDouble(gameManager.getTimeToNextShift());
		elasticTable.getEntry("SwerveYawDegrees").setDouble(swerve.getYaw().getDegrees());
		elasticTable.getEntry("LeftCanShoot").setBoolean(leftTurret.isAbleToShoot());
		elasticTable.getEntry("RightCanShoot").setBoolean(rightTurret.isAbleToShoot());
		elasticTable.getEntry("LeftCanAim").setBoolean(leftTurret.canAim());
		elasticTable.getEntry("RightCanAim").setBoolean(rightTurret.canAim());
	}

	public void close() {
		vortex.closeJetsons();
	}

	public void zeroMechanisms() {
		leftTurret.zeroPosition();
		rightTurret.zeroPosition();
		hopper.zeroPosition();
	}

	private static double nonInvSquare(double axis) {
		double deadbanded = MathUtil.applyDeadband(axis, Constants.Operator.kStickDeadband);
		double squared = Math.abs(deadbanded) * deadbanded;
		return squared;
	}
  
}
