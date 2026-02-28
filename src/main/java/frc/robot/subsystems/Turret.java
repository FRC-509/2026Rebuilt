package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Chassis.TurretConfiguration;
import frc.robot.util.Translation2dSupplier;

public class Turret extends SubsystemBase {
    
    private final TalonFX kRotationMotor;
    private final TalonFX kTopFlywheelMotor;
    private final TalonFX kBottomFlywheelMotor;

    private final VelocityDutyCycle kVelocityDutyCycle = new VelocityDutyCycle(0.0d);
    private final PositionDutyCycle kPositionDutyCycle = new PositionDutyCycle(0.0d);
    private final VoltageOut kVoltageOut = new VoltageOut(0.0d);

    private final Translation3d offsetTranslation;
    private final double maxRotationClockwise;
    private final double maxRotationCounterclockwise;

    private final Translation2dSupplier positionEstimate;
    private final DoubleSupplier robotYawRadiansSupplier;

    private AimTarget aimTarget;
    private double angleToTarget;
    private boolean canAim;
    private boolean overrideAimTarget;
    private double targetBottomFlywheelSpeed;
    private double targetTopFlywheelSpeed;

    private boolean hasZeroedPosition;
    private double zeroedRotationOffset;

    public Turret(
            TurretConfiguration turretConfiguration,
            Translation2dSupplier positionEstimate,
            DoubleSupplier robotYawDegreesSupplier) {
        this.kRotationMotor = new TalonFX(turretConfiguration.rotationMotorId());
        this.kTopFlywheelMotor = new TalonFX(turretConfiguration.topFlywheelMotorId());
        this.kBottomFlywheelMotor = new TalonFX(turretConfiguration.bottomFlywheelMotorId());

        // Rotation Motor Config
        TalonFXConfiguration rotationMotorConfig = new TalonFXConfiguration();
		rotationMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		rotationMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		rotationMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		rotationMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
		rotationMotorConfig.Feedback.SensorToMechanismRatio = Constants.Turret.kRotationMotorToMechanismRatio;

		rotationMotorConfig.Slot0.kP = Constants.PIDConstants.Turret.kRotationP;
		rotationMotorConfig.Slot0.kI = Constants.PIDConstants.Turret.kRotationI;
		rotationMotorConfig.Slot0.kD = Constants.PIDConstants.Turret.kRotationD;


		rotationMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		rotationMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kTurretRotationSupply; 
        rotationMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rotationMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kTurretRotationStator;

        this.kRotationMotor.getConfigurator().apply(rotationMotorConfig);


        // Flywheel motor config
        TalonFXConfiguration flywheelMotorConfig = new TalonFXConfiguration();
		flywheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		flywheelMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		flywheelMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		flywheelMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
		flywheelMotorConfig.Feedback.SensorToMechanismRatio = Constants.Turret.kFlywheelMotorToMechanismRatio;

		flywheelMotorConfig.Slot0.kP = Constants.PIDConstants.Turret.kFlywheelP;
		flywheelMotorConfig.Slot0.kI = Constants.PIDConstants.Turret.kFlywheelI;
		flywheelMotorConfig.Slot0.kD = Constants.PIDConstants.Turret.kFlywheelD;

		flywheelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kTurretFlywheelSupply; 
        flywheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kTurretFlywheelStator;

        this.kTopFlywheelMotor.getConfigurator().apply(flywheelMotorConfig);
        this.kBottomFlywheelMotor.getConfigurator().apply(flywheelMotorConfig);

        this.positionEstimate = positionEstimate;
        this.offsetTranslation = turretConfiguration.offsetTranslation();
        this.maxRotationClockwise = turretConfiguration.maxRotationClockwise();
        this.maxRotationCounterclockwise = turretConfiguration.maxRotationCounterclockwise();
        this.robotYawRadiansSupplier = robotYawDegreesSupplier;

        this.aimTarget = AimTarget.HOPPER;
        this.angleToTarget = 0;
        this.overrideAimTarget = false;
        this.targetBottomFlywheelSpeed = 0; // change this to spin up at start of match?
        this.targetTopFlywheelSpeed = 0;
        this.canAim = false;

        this.hasZeroedPosition = false;
        this.zeroedRotationOffset = this.kRotationMotor.getPosition().getValueAsDouble(); // temp (?) to assume zeroed on initialize
    }
 
    public enum AimTarget {

        NONE(Translation3d.kZero, 0),
        HOPPER(new Translation3d(), 0.1),
        NEUTRALZONE_FEED_LEFT(new Translation3d(),0),
        NEUTRALZONE_FEED_RIGHT(new Translation3d(), 0),
        OPPOSING_ALLIANCE_FEED_LEFT(new Translation3d(),0),
        OPPOSING_ALLIANCE_FEED_RIGHT(new Translation3d(), 0);

        public final Translation3d position;
        public final double aimBehindMeters;

        private AimTarget(Translation3d position, double aimBehindMeters) {
            this.position = position;
            this.aimBehindMeters = aimBehindMeters;
        }

        public Translation3d aimAccountedTarget(double swerveYawRadians) { // aim slightly behind target for accuracy
            return new Translation3d(
                position.getX() + aimBehindMeters * Math.cos(swerveYawRadians),
                position.getY() + aimBehindMeters * -Math.sin(swerveYawRadians),
                position.getZ()
            );
        }
    }

    private void setRotation(double degrees) {
        angleToTarget = degrees / Constants.Turret.kRotationToTurretDegrees - zeroedRotationOffset; // TODO: change to actual conversion
        kRotationMotor.setControl(kPositionDutyCycle.withPosition(angleToTarget));
    }

    private double getRotationDegrees() {
        return (kRotationMotor.getPosition().getValueAsDouble() - zeroedRotationOffset) * Constants.Turret.kRotationToTurretDegrees;
    }

    public void setAimTarget(AimTarget aimTarget) {
        this.aimTarget = aimTarget;
    }

    public Translation2d getTurretGlobalPosition() {
        return positionEstimate.getAsTranslation2d().plus(
            new Translation2d(
                offsetTranslation.getX() * Math.cos(robotYawRadiansSupplier.getAsDouble()) + offsetTranslation.getY() * Math.sin(robotYawRadiansSupplier.getAsDouble()),
                offsetTranslation.getX() * -Math.sin(robotYawRadiansSupplier.getAsDouble()) + offsetTranslation.getY() * Math.cos(robotYawRadiansSupplier.getAsDouble())
            ));
    }

    public double getRotationToTarget(AimTarget targetPosition) {
        Translation2d targetTurretRelative = targetPosition.position.toTranslation2d().minus(getTurretGlobalPosition());
        double angle = Math.atan2(targetTurretRelative.getY(), targetTurretRelative.getX()) - robotYawRadiansSupplier.getAsDouble(); // TODO: confirm minus
        return angle;
    }

    public boolean isAbleToShoot() {
        return canAim
            && MathUtil.isNear(angleToTarget, getRotationDegrees(), Constants.Turret.kRotationTolerance)
            && MathUtil.isNear(targetBottomFlywheelSpeed, kBottomFlywheelMotor.getVelocity().getValueAsDouble(), Constants.Turret.kFlywheelSpeedTolerance)
            && MathUtil.isNear(targetTopFlywheelSpeed, kTopFlywheelMotor.getVelocity().getValueAsDouble(), Constants.Turret.kFlywheelSpeedTolerance);
    }

    public void setOverrideAimTarget(boolean override) {
        this.overrideAimTarget = override;
    }

    private AimTarget getTargetFromPosition() {
        /* Field orientation
        * ^
	    * | X+
	    * ----> Y-
        */
        Translation2d turretGlobal = getTurretGlobalPosition();
        Translation2d turretRelative = DriverStation.getAlliance().get().equals(Alliance.Red) //TODO: confirm blue alliance has 0,0
            ? new Translation2d(Constants.Field.kFullFieldLength,Constants.Field.kFieldWidth).minus(turretGlobal)
            : turretGlobal;

        if (turretRelative.getX() > Constants.Field.kAllianceZoneLength + Constants.Field.kNeutralZoneLength) {
            if (turretRelative.getY() > Constants.Field.kFieldWidth/2) return AimTarget.OPPOSING_ALLIANCE_FEED_LEFT;
            return AimTarget.OPPOSING_ALLIANCE_FEED_RIGHT;
        } 
        if (turretRelative.getX() > Constants.Field.kAllianceZoneLength + Constants.Field.kNeutralZoneLength) {
            if (turretRelative.getY() > Constants.Field.kFieldWidth/2) return AimTarget.NEUTRALZONE_FEED_LEFT;
            return AimTarget.NEUTRALZONE_FEED_RIGHT;
        }
        return AimTarget.HOPPER;
    }

    private double calculateFlywheelSpeeds() {
        Translation2d turretGlobal = getTurretGlobalPosition();
        Translation3d targetTurretRelative = aimTarget.aimAccountedTarget(robotYawRadiansSupplier.getAsDouble()).minus(new Translation3d(turretGlobal.getX(), turretGlobal.getY(), Constants.Turret.kTurretHeightFromGround));

        double theta = Math.toRadians(90 - Constants.Turret.kTurretAngleDegrees);
        double cosT = Math.cos(theta);
        
        double dist = targetTurretRelative.toTranslation2d().getDistance(Translation2d.kZero);
        double denom = (dist * Math.tan(theta)) - targetTurretRelative.getZ();
        if (!(denom > 0 && Math.abs(cosT) > 0.001)) return 0d;

        double exitVelocity = MathUtil.clamp(
            Math.sqrt((9.8 * dist * dist) / (2 * cosT * cosT * denom)),
            0.0d,
            Constants.Turret.kMaxExitVelocity); // divide by efficiency?
        
        double angularVelocityRadPerSec = exitVelocity / Constants.Turret.kFlywheelRadiusMeters;
        return angularVelocityRadPerSec / 2 / Math.PI;
    }

    private double[] calculateFlywheelSpeedsGeff() { // solves for aim with spin + approximation of magnus effect
        Translation2d turretGlobal = getTurretGlobalPosition();
        Translation3d targetTurretRelative = aimTarget.aimAccountedTarget(robotYawRadiansSupplier.getAsDouble()).minus(
            new Translation3d(turretGlobal.getX(), turretGlobal.getY(), Constants.Turret.kTurretHeightFromGround));

        double dist = targetTurretRelative.toTranslation2d().getDistance(Translation2d.kZero);
        double targetZ = targetTurretRelative.getZ();
        double theta = Math.toRadians(90 - Constants.Turret.kTurretAngleDegrees);
    
        double liftAcceleration = Constants.Turret.kMagnusCoefficient * Constants.Turret.kTargetSpinRadSec; // magnus effect approximationn
        double gEff = 9.8 - liftAcceleration;
        double cosT = Math.cos(theta);
        double denom = (dist * Math.tan(theta)) - targetZ;
        if (!(denom > 0 && Math.abs(cosT) > 0.001)) return new double[] {0, 0};

        double exitVelocity = MathUtil.clamp(
            Math.sqrt((gEff * dist * dist) / (2 * cosT * cosT * denom)),
            0.0d, Constants.Turret.kMaxExitVelocity);

        // v_exit = (Vb + Vt)/2, Spin_surface = (Vb - Vt)/2, therefore: Vb = V_exit + Spin_surface
        double surfaceSpeedDiff = Constants.Turret.kTargetSpinRadSec * Constants.Field.kFuelRadiusMeters;
        double vBottom = (exitVelocity + surfaceSpeedDiff) / Constants.Turret.kEfficiency;
        double vTop = (exitVelocity - surfaceSpeedDiff) / Constants.Turret.kEfficiency;

        double circ = 2 * Math.PI * Constants.Turret.kFlywheelRadiusMeters;
        return new double[] { vBottom / circ, vTop / circ };
    }

    private double[] calculateSpeedsManualMagnus() {
        Translation2d turretGlobal = getTurretGlobalPosition();
        Translation3d targetTurretRelative = aimTarget.aimAccountedTarget(robotYawRadiansSupplier.getAsDouble()).minus(
            new Translation3d(turretGlobal.getX(), turretGlobal.getY(), Constants.Turret.kTurretHeightFromGround));

        double dist = targetTurretRelative.toTranslation2d().getDistance(Translation2d.kZero);
        double targetZ = targetTurretRelative.getZ();
        double theta = Math.toRadians(90 - Constants.Turret.kTurretAngleDegrees);
        double cosT = Math.cos(theta);
        double initialV = Math.sqrt((9.8 * dist * dist) / (2 * cosT * cosT * ((dist * Math.tan(theta)) - targetZ)));

        // magnus force = 1/2 * rho * A * liftCoeff * v^2   (liftCoeff for a sphere is often approximated as (r * spin / v))
        double airDensity = 1.225;
        double crossSectionArea = Math.PI * Math.pow(Constants.Field.kFuelRadiusMeters, 2);
        double spinRatio = (Constants.Field.kFuelRadiusMeters * Constants.Turret.kTargetSpinRadSec) / initialV;
        double liftCoefficient = 1.5 * spinRatio; 
        
        double forceLift = 0.5 * airDensity * Math.pow(initialV, 2) * crossSectionArea * liftCoefficient;
        double accelLift = forceLift / Constants.Turret.kAverageFuelMass;

        double denom = (dist * Math.tan(theta)) - targetZ;
        double finalExitVelocity = Math.sqrt(((9.8 - accelLift) * dist * dist) / (2 * cosT * cosT * denom));

        // split between flywheels
        double surfaceSpeedDiff = Constants.Turret.kTargetSpinRadSec * Constants.Field.kFuelRadiusMeters;
        double vBottom = (finalExitVelocity + surfaceSpeedDiff) / Constants.Turret.kEfficiency;
        double vTop = (finalExitVelocity - surfaceSpeedDiff) / Constants.Turret.kEfficiency;

        double circ = 2 * Math.PI * Constants.Turret.kFlywheelRadiusMeters;
        return new double[] { vBottom / circ, vTop / circ };
    }

    @Override
    public void periodic() {
        // TODO: replace with non hard stop version, complient with design change
        // zeroing functionality to move until you hit minimum hardstop
        if (!hasZeroedPosition) {
            // TODO: double check this method works and isn't cancelled immediately, also add needed tolerances
            if (kRotationMotor.getVelocity().getValueAsDouble() == 0 && kRotationMotor.getAppliedControl().getClass().equals(kVoltageOut.getClass())) {
                kRotationMotor.setControl(kVoltageOut.withOutput(0));
                zeroedRotationOffset = kRotationMotor.getPosition().getValueAsDouble();
                hasZeroedPosition = true;
            } else kRotationMotor.setControl(kVoltageOut.withOutput(1)); // TODO: change zeroing voltage

            if (!hasZeroedPosition) return;
        }


        // decide on target based on position
        if (!overrideAimTarget) aimTarget = getTargetFromPosition(); // override needed?
        angleToTarget = getRotationToTarget(aimTarget);
        this.canAim = !aimTarget.equals(AimTarget.NONE) && angleToTarget <= maxRotationClockwise && angleToTarget >= maxRotationCounterclockwise; // check if desired angle is within bounds // TODO: double check which direction is positive or negative
        setRotation(canAim ? angleToTarget : 0);

        // decide flywheel speed every 0.02s (ie always) for target
        if (canAim) {
            double[] flywheelSpeeds = calculateSpeedsManualMagnus();
            kBottomFlywheelMotor.setControl(kVelocityDutyCycle.withVelocity(MathUtil.clamp(flywheelSpeeds[0], 0d, 100d)));
            kTopFlywheelMotor.setControl(kVelocityDutyCycle.withVelocity(MathUtil.clamp(flywheelSpeeds[1], 0d, 100d)));
        }
    }

    public void zeroPosition() {
        this.hasZeroedPosition = false;
    }
}
