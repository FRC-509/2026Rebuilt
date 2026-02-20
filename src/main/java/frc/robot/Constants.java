// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.Conversions;

public final class Constants {
	public static final String kRio = "rio";

	public static class Operator {
		public static final double kStickDeadband = 0.1;
        public static final double kPrecisionMovementMultiplier = 0.3;
		public static final double kPrecisionRotationMultiplier = 0.2;
		public static final double kTriggerDeadband = 0.15;
	}

	public static class Chassis {
		// TODO: Change all
		public static final double kRobotWeight = 61.7d;
		public static final double kMOI = 5.2013;
		public static final double kOffsetToSwerveModule = 0.395;
		public static final double kKrakenFreeSpeedRPM = 6000.0d;
		public static final double kKrakenFreeSpeedRPS = kKrakenFreeSpeedRPM / 60.0d;
		public static final double kMaxSpeed = Conversions.falconToMPS(kKrakenFreeSpeedRPS, N5.kWheelCircumference,
			N5.kDriveGearRatio); // test
		
		public static class N5 { // R2 config
			public static final double kWheelRadius = Units.inchesToMeters(2.0);
			public static final double kWheelCircumference = 2 * kWheelRadius * Math.PI; // 0.3192 meters
			public static final double kDriveGearRatio = 6.03 / 1;
			public static final double kAngleGearRatio = 287.0d / 11.0d;
			public static final double kCouplingRatio = 25.0d / 7.0d;
			public static final double wheelCOF = 1.0; // default placeholder value
		}

		public static final double kMaxAngularVelocity = kMaxSpeed
			/ (Math.hypot(Chassis.kOffsetToSwerveModule, Chassis.kOffsetToSwerveModule));
		// public static final double kMaxAngularAcceleration = 0.0;

		public static final DCMotor kKrakenDcMotorProfile = new DCMotor(
			12,
			7.09, 
			366, 
			2,
			kKrakenFreeSpeedRPS * 2 * Math.PI,
			1);

		public static record SwerveModuleConfiguration(
			int moduleNumber,
			int steerEncoderId,
			int steerMotorId,
			int driveMotorId,
			double steerEncoderOffset) {}

		public static final SwerveModuleConfiguration kFrontRight = new SwerveModuleConfiguration(
			0,
			IDs.kFrontRightEncoder,
			IDs.kFrontRightSteer,
			IDs.kFrontRightDrive,
			118.587891);

		public static final SwerveModuleConfiguration kFrontLeft = new SwerveModuleConfiguration(
			1,
			IDs.kFrontLeftEncoder,
			IDs.kFrontLeftSteer,
			IDs.kFrontLeftDrive,
			153.337891);

		public static final SwerveModuleConfiguration kBackLeft = new SwerveModuleConfiguration(
			2,
			IDs.kBackLeftEncoder,
			IDs.kBackLeftSteer,
			IDs.kBackLeftDrive,
			104.066406+1);

		public static final SwerveModuleConfiguration kBackRight = new SwerveModuleConfiguration(
			3,
			IDs.kBackRightEncoder,
			IDs.kBackRightSteer,
			IDs.kBackRightDrive,
			140.425782);

		public static final double kRobotWidth = Units.inchesToMeters(26);
		public static final double kBumperWidth = Units.inchesToMeters(3.5);
        public static final double kSafePathingTolerance = 0; // TODO: set me pretty please
        public static final double kValidPositionTolerance = 0;
        public static final double kValidHeadingTolerance = 0;

		public static final double kMaxAcceleration = 0;
        public static final double kMaxDecceleration = 0;
	}

	public static class Turret {
		// TODO: find me
		public static final double kRotationMotorToMechanismRatio = 0.0d;
		public static final double kFlywheelMotorToMechanismRatio = 0.0d;

		public static final double kRotationTolerance = 0.25; // degrees
		public static final double kFlywheelSpeedTolerance = 25; // rpm

        public static final double kTurretHeightFromGround = 0;
        public static final double kTurretAngleDegrees = 17;
		public static final double kMaxExitVelocity = 0;

		public static final double kFlywheelRadiusMeters = 0;
		public static final double kFlywheelMass = 0;
        public static final double kFlywheelMOI = 0.5 * kFlywheelMass * kFlywheelRadiusMeters * kFlywheelRadiusMeters;

        public static final double kAverageFuelMass = 0;
        public static final double kTargetSpinRadSec = 0;

        public static final double kEfficiency = 0;
        public static final double kMagnusCoefficient = 0.02; // tune 0.02~0.05

        public static class LeftTurret {
            public static final Translation3d kLeftTurretOffset = new Translation3d();
            public static final double kMaxRotationClockwiseDegrees = 0;
			public static final double kMaxRotationCounterClockwiseDegrees = 0;
		}
		
        public static class RightTurret {
            public static final Translation3d kRightTurretOffset = new Translation3d();
            public static final double kMaxRotationClockwiseDegrees = 0;
			public static final double kMaxRotationCounterClockwiseDegrees = 0;
		}
	}

	public static class Hopper { // TODO: find me
        public static final double kIntakingVelocity = 0.0;
        public static final double kOuttakingVelocity = 0.0;
		public static final double kIndexerRollersVelocity = 0.0;
		public static final double kIndexingVelocity = 0.0;
		
		public static final double kIntakeExtension = 0.0;
        public static final double kIntakeExtensionToMetersConversion = 0;
	}

	public static class IDs {
		// Swerve Drive
		public static final int kFrontRightDrive = 11;
		public static final int kFrontRightSteer = 9;
		public static final int kFrontRightEncoder = 3;

		public static final int kFrontLeftDrive = 10;
		public static final int kFrontLeftSteer = 6;
		public static final int kFrontLeftEncoder = 1;
		
		public static final int kBackRightDrive = 8;
		public static final int kBackRightSteer = 5;
		public static final int kBackRightEncoder = 4;
		
		public static final int kBackLeftDrive = 12;
		public static final int kBackLeftSteer = 7;
		public static final int kBackLeftEncoder = 2;

		// Shooters
		public static final int kLeftBottomFlywheel = 0;
		public static final int kLeftTopFlywheel = 0;
		public static final int kLeftRotationMotor = 0;
		
		public static final int kRightBottomFlywheel = 0;
		public static final int kRightTopFlywheel = 0;
		public static final int kRightRotationMotor = 0;
		
		// Hopper
        public static final int kIntakeExtension = 0;
        public static final int kIntakeRotation = 0;
        public static final int kIndexerRotation = 0;
	}

	public static class PathGeneration {
		public static final double kSafePathTolerance = 0.2; // m
        public static final double kAdjustRate = 0.01;
        public static final double kCurvaturePointRemovalRadius = 0; // m
		public static final double kRetryPathingDelay = 0.2;
        public static final int kLengthApproximationSegments = 25;
		public static final double kStoppingDistance = 0.5; //m
	}

	public static class PIDConstants {
		public static class Drive {
			// TODO: Tune Me
			public static final double kDriveVelocityP = 0.2;
			public static final double kDriveVelocityI = 3.0;
			public static final double kDriveVelocityD = 0.0;
			public static final double kDriveVelocityS = 0.124;
			public static final double kDriveVelocityV = 0.109;
			public static final double kDriveVelocityA = 0.0;

			public static final double kSteerAngleP = 100.0;
			public static final double kSteerAngleI = 0.0;
			public static final double kSteerAngleD = 0.0;

			public static final double kHeadingPassiveP = 1.0;
			public static final double kHeadingPassiveI = 0.15;
			public static final double kHeadingPassiveD = 0.0;
			public static final double kHeadingAggressiveP = 4.5;
			public static final double kHeadingAggressiveI = 0.25;
			public static final double kHeadingAggressiveD = 0.0;
			public static final double kHeadingTimeout = 0.25;
			public static final double kMinHeadingCorrectionSpeed = 0.1;
		}

		public static class Turret {
            public static final double kRotationP = 0;
            public static final double kRotationI = 0;
            public static final double kRotationD = 0;

            public static final double kFlywheelP = 0;
            public static final double kFlywheelI = 0;
            public static final double kFlywheelD = 0;
		}

		public static class Hopper {
            public static final double kExtensionP = 0;
            public static final double kExtensionI = 0;
            public static final double kExtensionD = 0;

			public static final double kIntakeP = 0;
            public static final double kIntakeI = 0;
            public static final double kIntakeD = 0;
			
			public static final double kIndexerP = 0;
            public static final double kIndexerI = 0;
            public static final double kIndexerD = 0;

            public static final double kIndexerRollersP = 0;
            public static final double kIndexerRollersI = 0;
            public static final double kIndexerRollersD = 0;
		}
	}

	public static class CurrentLimits {
		public static final double kSwerveModuleSupply = 35.0d;
		// SwerveStator ?

        public static final double kTurretRotationSupply = 0;
        public static final double kTurretRotationStator = 0;

		public static final double kTurretFlywheelSupply = 0;
        public static final double kTurretFlywheelStator = 0;

		public static final double kIntakeExtensionSupply = 0;
        public static final double kIntakeExtensionStator = 0;

		public static final double kIntakeSupply = 0;
        public static final double kIntakeStator = 0;

        public static final double kIndexerSupply = 0;
        public static final double kIndexerStator = 0;

        public static final double kIndexerRollersSupply = 0;

        public static final double kIndexerRollersStator = 0;
	}

	public static class Field {
		public static final double kFullFieldLength = 16.54d; // Double check
        public static final double kFieldWidth = 8.1;
        public static final double kAllianceZoneLength = 0.0d; // TODO: find
        public static final double kNeutralZoneLength = kFullFieldLength - 2 * kAllianceZoneLength;
		
        public static final double kAverageFuelMass = 0;
		public static final double kFuelRadiusMeters = 0;
	}

	public static double tunableNumber(String name, double defaultValue){
		return SmartDashboard.getNumber(name, defaultValue);
	}
}
