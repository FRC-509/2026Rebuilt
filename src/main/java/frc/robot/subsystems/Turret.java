package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Chassis.TurretConfiguration;
import frc.robot.subsystems.drive.SwerveDrive;
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
    private final boolean zeroesCounterClockwise;
    private final double zeroedRotationMaximumAdded;
    private final String side;

    private final Translation2dSupplier positionEstimate;
    private final Translation2dSupplier robotVelocitySupplier;
    private final DoubleSupplier robotYawRadiansSupplier;
    private final BooleanSupplier isIndexingSupplier;

    private AimTarget aimTarget;
    private double targetRotationDegrees;
    private double targetRotationMotorPosition;
    private boolean canAim;
    private boolean overrideAimTarget;
    private boolean overridePositionEstimate;
    private Translation2d overriddenPositionEstimate;
    private double targetBottomFlywheelSpeed;
    private double targetTopFlywheelSpeed;

    private boolean hasZeroedPosition;
    private double zeroedRotationOffset;

    public Turret(
            TurretConfiguration turretConfiguration,
            Translation2dSupplier positionEstimate,
            Translation2dSupplier robotVelocitySupplier,
            DoubleSupplier robotYawSupplier,
            BooleanSupplier isIndexingSupplier) {
        this.kRotationMotor = new TalonFX(turretConfiguration.rotationMotorId(), Constants.kCanivore);
        this.kTopFlywheelMotor = new TalonFX(turretConfiguration.topFlywheelMotorId(), Constants.kCanivore);
        this.kBottomFlywheelMotor = new TalonFX(turretConfiguration.bottomFlywheelMotorId(), Constants.kCanivore);

        // Rotation Motor Config
        TalonFXConfiguration rotationMotorConfig = new TalonFXConfiguration();
		rotationMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		rotationMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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

        rotationMotorConfig.Voltage.PeakForwardVoltage = 5;
        rotationMotorConfig.Voltage.PeakReverseVoltage = -5;

        this.kRotationMotor.getConfigurator().apply(rotationMotorConfig);


        // Flywheel motor config
        TalonFXConfiguration flywheelMotorConfig = new TalonFXConfiguration();
		flywheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		flywheelMotorConfig.MotorOutput.Inverted = turretConfiguration.zeroesCounterClockwise() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		flywheelMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		flywheelMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

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
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.offsetTranslation = turretConfiguration.offsetTranslation();
        this.maxRotationClockwise = turretConfiguration.maxRotationClockwise();
        this.maxRotationCounterclockwise = turretConfiguration.maxRotationCounterclockwise();
        this.robotYawRadiansSupplier = robotYawSupplier;
        this.isIndexingSupplier = isIndexingSupplier;

        this.aimTarget = AimTarget.HUB;
        this.targetRotationDegrees = 0;
        this.targetRotationMotorPosition = 0;
        this.overrideAimTarget = false;
        this.overridePositionEstimate = false;
        this.overriddenPositionEstimate = Translation2d.kZero;
        this.targetBottomFlywheelSpeed = 0; // change this to spin up at start of match?
        this.targetTopFlywheelSpeed = 0;
        this.canAim = false;

        this.hasZeroedPosition = false;
        this.zeroedRotationOffset = this.kRotationMotor.getPosition().getValueAsDouble(); // temp (?) to assume zeroed on initialize
        this.zeroesCounterClockwise = turretConfiguration.zeroesCounterClockwise();
        this.zeroedRotationMaximumAdded = zeroesCounterClockwise ? maxRotationCounterclockwise : maxRotationClockwise;
        this.side = turretConfiguration.side();
    }
 
    public enum AimTarget {

        NONE(Translation3d.kZero, 0, 0),
        HUB(new Translation3d(5.4,Constants.Field.kFieldWidth/2,1.88), 0.125, 1.8),
        NEUTRALZONE_FEED_LEFT(new Translation3d(),0, 0),
        NEUTRALZONE_FEED_RIGHT(new Translation3d(), 0, 0),
        OPPOSING_ALLIANCE_FEED_LEFT(new Translation3d(),0, 0),
        OPPOSING_ALLIANCE_FEED_RIGHT(new Translation3d(), 0, 0);

        public final Translation3d position;
        public final double aimBehindMeters;
        public final double targetBackspinRadSec;

        private AimTarget(Translation3d position, double aimBehindMeters, double targetBackspinRadSec) {
            this.position = position;
            this.aimBehindMeters = aimBehindMeters;
            this.targetBackspinRadSec = targetBackspinRadSec;
        }

        public Translation3d aimAccountedTarget(double swerveYawRadians, Translation2d robotVelocity) { // aim slightly behind target for accuracy, and account for chassis movement
            return new Translation3d(
                position.getX() + aimBehindMeters * Math.cos(swerveYawRadians) - robotVelocity.getX() * Constants.Turret.kMovementCorrectionConstant,
                position.getY() + aimBehindMeters * Math.sin(swerveYawRadians) - robotVelocity.getY() * Constants.Turret.kMovementCorrectionConstant,
                position.getZ()
            );
        }
    }

    private void setRotationDegrees(double degrees) {
        targetRotationDegrees = degrees;
        targetRotationMotorPosition = (degrees - zeroedRotationMaximumAdded) / 360 + zeroedRotationOffset;
        kRotationMotor.setControl(kPositionDutyCycle.withPosition(targetRotationMotorPosition));
    }

    private double getRotationDegrees() {
        return (kRotationMotor.getPosition().getValueAsDouble() - zeroedRotationOffset) * 360 + zeroedRotationMaximumAdded;
    }

    public double getCurrentRotationDegrees() {
        return getRotationDegrees();
    }

    public void setAimTarget(AimTarget aimTarget) {
        this.aimTarget = aimTarget;
    }

    public void setShootSpeed(boolean isIndexing){

    }

    public Translation2d getTurretAlliancePosition() {
        double yawRadians = getTurretYawRadians();
        Translation2d robotPosition = overridePositionEstimate
            ? overriddenPositionEstimate
            : positionEstimate.getAsTranslation2d();
        return robotPosition.plus(
            new Translation2d(
                offsetTranslation.getX() * Math.cos(yawRadians) - offsetTranslation.getY() * Math.sin(yawRadians),
                offsetTranslation.getX() * Math.sin(yawRadians) + offsetTranslation.getY() * Math.cos(yawRadians)
            ));
    }

    public Pose2d getTurretAlliancePose() {
        double headingDegrees = Math.toDegrees(getTurretYawRadians()) + getRotationDegrees() - 180.0;
        return new Pose2d(getTurretAlliancePosition(), Rotation2d.fromDegrees(headingDegrees));
    }

    public Pose2d getTurretFieldPose() {
        return getTurretAlliancePose();
    }

    public double getRotationToTarget(AimTarget targetPosition) {
        Translation2d turretPosition = getTurretAlliancePosition();
        Translation2d targetTurretRelative = getTargetFieldPosition(targetPosition, turretPosition, getTurretYawRadians(), robotVelocitySupplier.getAsTranslation2d())
            .toTranslation2d()
            .minus(turretPosition);
        return Math.toDegrees(MathUtil.angleModulus(
            Math.atan2(targetTurretRelative.getY(), targetTurretRelative.getX())
                - getTurretYawRadians()));
    }

    public boolean isAbleToShoot() {
        return canAim
            && MathUtil.isNear(targetRotationDegrees, getRotationDegrees(), Constants.Turret.kRotationTolerance)
            && isFlywheelAtTargetSpeed(kBottomFlywheelMotor, targetBottomFlywheelSpeed)
            && isFlywheelAtTargetSpeed(kTopFlywheelMotor, targetTopFlywheelSpeed);
    }

    public boolean isShooterUpToSpeed() {
        return MathUtil.isNear(
                Math.abs(targetBottomFlywheelSpeed),
                Math.abs(kBottomFlywheelMotor.getVelocity().getValueAsDouble()),
                Constants.Turret.kIndexerFeedFlywheelToleranceRps)
            && MathUtil.isNear(
                Math.abs(targetTopFlywheelSpeed),
                Math.abs(kTopFlywheelMotor.getVelocity().getValueAsDouble()),
                Constants.Turret.kIndexerFeedFlywheelToleranceRps);
    }

    public boolean canAim() {
        return canAim;
    }

    public void setOverrideAimTarget(boolean override, AimTarget target) {
        this.overrideAimTarget = override;
        if (override) this.aimTarget = target;
    }

    public void setOverridePositionEstimate(Translation2d positionEstimate) {
        overridePositionEstimate = true;
        overriddenPositionEstimate = positionEstimate;
    }

    public void clearOverridePositionEstimate() {
        overridePositionEstimate = false;
    }

    private AimTarget getTargetFromPosition() {
        // Translation2d robotRelative = positionEstimate.getAsTranslation2d();
        Translation2d turretRelative = getTurretAlliancePosition();
        // if (Math.abs(robotRelative.getX() - AimTarget.HUB.position.getX()) <= Constants.Chassis.kRobotWidth) {
        //     return AimTarget.HUB;
        // }

        double neutralZoneStart = Constants.Field.kAllianceZoneLength;
        double opposingAllianceStart = neutralZoneStart + Constants.Field.kNeutralZoneLength;

        AimTarget selectedTarget = AimTarget.HUB;
        if (turretRelative.getX() > opposingAllianceStart) {
            selectedTarget = turretRelative.getY() > Constants.Field.kFieldWidth / 2
                ? AimTarget.OPPOSING_ALLIANCE_FEED_LEFT
                : AimTarget.OPPOSING_ALLIANCE_FEED_RIGHT;
        } else if (turretRelative.getX() > neutralZoneStart) {
            selectedTarget = turretRelative.getY() > Constants.Field.kFieldWidth / 2
                ? AimTarget.NEUTRALZONE_FEED_LEFT
                : AimTarget.NEUTRALZONE_FEED_RIGHT;
        }

        return hasConfiguredPosition(selectedTarget) ? selectedTarget : AimTarget.HUB;
    }

    private boolean hasConfiguredPosition(AimTarget target) {
        return target == AimTarget.HUB || target.position.getNorm() > 0.001;
    }

    private boolean isFlywheelAtTargetSpeed(TalonFX motor, double targetVelocity) {
        return MathUtil.isNear(
            Math.abs(targetVelocity),
            Math.abs(motor.getVelocity().getValueAsDouble()),
            Constants.Turret.kFlywheelSpeedTolerance);
    }

    private Translation3d getTargetTurretRelative() {
        Translation2d turretPosition = getTurretAlliancePosition();
        return getTargetFieldPosition(aimTarget, turretPosition, getTurretYawRadians(), robotVelocitySupplier.getAsTranslation2d()).minus(
            new Translation3d(turretPosition.getX(), turretPosition.getY(), offsetTranslation.getZ()));
    }

    private double getTurretYawRadians() {
        return robotYawRadiansSupplier.getAsDouble();
    }

    private Translation3d getTargetFieldPosition(AimTarget target) {
        return getTargetFieldPosition(target, getTurretAlliancePosition(), 0.0, Translation2d.kZero);
    }

    private Translation3d getTargetFieldPosition(AimTarget target, Translation2d turretPosition, double yawRadians, Translation2d robotVelocity) {
        Translation3d targetPosition = target.aimAccountedTarget(yawRadians, robotVelocity);
        if (SwerveDrive.getAlliance() != edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            return resolveCloserHubTarget(target, turretPosition, targetPosition);
        }

        Translation3d mirroredTargetPosition = new Translation3d(
            Constants.Field.kFullFieldLength - targetPosition.getX(),
            targetPosition.getY(),
            targetPosition.getZ());
        return resolveCloserHubTarget(target, turretPosition, mirroredTargetPosition);
    }

    private Translation3d resolveCloserHubTarget(AimTarget target, Translation2d turretPosition, Translation3d candidateTargetPosition) {
        if (target != AimTarget.HUB) {
            return candidateTargetPosition;
        }

        Translation3d oppositeHubTargetPosition = new Translation3d(
            Constants.Field.kFullFieldLength - candidateTargetPosition.getX(),
            candidateTargetPosition.getY(),
            candidateTargetPosition.getZ());

        double candidateDistance = turretPosition.getDistance(candidateTargetPosition.toTranslation2d());
        double oppositeDistance = turretPosition.getDistance(oppositeHubTargetPosition.toTranslation2d());
        return candidateDistance <= oppositeDistance ? candidateTargetPosition : oppositeHubTargetPosition;
    }

    private double calculateFlywheelSpeeds() {
        Translation3d targetTurretRelative = getTargetTurretRelative();
        double theta = Math.toRadians(90 - Constants.Turret.kTurretAngleDegrees);
        double cosT = Math.cos(theta);
        
        double dist = targetTurretRelative.toTranslation2d().getDistance(Translation2d.kZero);
        double denom = (dist * Math.tan(theta)) - targetTurretRelative.getZ();
        if (!(denom > 0 && Math.abs(cosT) > 0.001)) return 0d;

        double exitVelocity = Math.sqrt((9.8 * dist * dist) / (2 * cosT * cosT * denom));
        
        double angularVelocityRadPerSec = exitVelocity / Constants.Turret.kFlywheelRadiusMeters;
        return MathUtil.clamp(angularVelocityRadPerSec / 2 / Math.PI, 0, 100);
    }

    private double[] calculateFlywheelSpeedsGeff() { // solves for aim with spin + approximation of magnus effect
        Translation3d targetTurretRelative = getTargetTurretRelative();
        double dist = targetTurretRelative.toTranslation2d().getDistance(Translation2d.kZero);
        double targetZ = targetTurretRelative.getZ();
        double theta = Math.toRadians(90 - Constants.Turret.kTurretAngleDegrees);
    
        double liftAcceleration = Constants.Turret.kMagnusCoefficient * aimTarget.targetBackspinRadSec; // magnus effect approximationn
        double gEff = 9.8 - liftAcceleration;
        double cosT = Math.cos(theta);
        double denom = (dist * Math.tan(theta)) - targetZ;
        if (!(denom > 0 && Math.abs(cosT) > 0.001)) return new double[] {0, 0};

        double exitVelocity = Math.sqrt((gEff * dist * dist) / (2 * cosT * cosT * denom));

        // v_exit = (Vb + Vt)/2, Spin_surface = (Vb - Vt)/2, therefore: Vb = V_exit + Spin_surface
        double surfaceSpeedDiff = aimTarget.targetBackspinRadSec * Constants.Field.kFuelRadiusMeters;
        double vBottom = (exitVelocity + surfaceSpeedDiff) / Constants.Turret.kEfficiency;
        double vTop = (exitVelocity - surfaceSpeedDiff) / Constants.Turret.kEfficiency;

        double circ = 2 * Math.PI * Constants.Turret.kFlywheelRadiusMeters;
        return new double[] { 
            MathUtil.clamp(vBottom / circ, 0, 100), 
            MathUtil.clamp(vTop / circ, 0, 100) 
        };
    }

    private double[] calculateSpeedsManualMagnus() {
        Translation3d targetTurretRelative = getTargetTurretRelative();
        double dist = targetTurretRelative.toTranslation2d().getDistance(Translation2d.kZero);
        double targetZ = targetTurretRelative.getZ();
        double theta = Math.toRadians(90 - Constants.Turret.kTurretAngleDegrees);
        double cosT = Math.cos(theta);
        double initialV = Math.sqrt((9.8 * dist * dist) / (2 * cosT * cosT * ((dist * Math.tan(theta)) - targetZ)));

        // magnus force = 1/2 * rho * A * liftCoeff * v^2   (liftCoeff for a sphere is often approximated as (r * spin / v))
        double airDensity = 1.225;
        double crossSectionArea = Math.PI * Math.pow(Constants.Field.kFuelRadiusMeters, 2);
        double spinRatio = (Constants.Field.kFuelRadiusMeters * aimTarget.targetBackspinRadSec) / initialV;
        double liftCoefficient = 1.5 * spinRatio; 
        
        double forceLift = 0.5 * airDensity * Math.pow(initialV, 2) * crossSectionArea * liftCoefficient;
        double accelLift = forceLift / Constants.Turret.kAverageFuelMass;

        double denom = (dist * Math.tan(theta)) - targetZ;
        double finalExitVelocity = Math.sqrt(((9.8 - accelLift) * dist * dist) / (2 * cosT * cosT * denom));

        // split between flywheels
        double surfaceSpeedDiff = aimTarget.targetBackspinRadSec * Constants.Field.kFuelRadiusMeters;
        double vBottom = (finalExitVelocity + surfaceSpeedDiff) / Constants.Turret.kEfficiency;
        double vTop = (finalExitVelocity - surfaceSpeedDiff) / Constants.Turret.kEfficiency;

        double circ = 2 * Math.PI * Constants.Turret.kFlywheelRadiusMeters;
        return new double[] { vBottom / circ, vTop / circ };
    }

    @Override
    public void periodic() {
        // zeroing functionality to move until you hit minimum hardstop
        if (!hasZeroedPosition) {
            if (Math.abs(kRotationMotor.getTorqueCurrent().getValueAsDouble()) > 35) {
                kRotationMotor.setControl(kVoltageOut.withOutput(0));
                zeroedRotationOffset = kRotationMotor.getPosition().getValueAsDouble();
                hasZeroedPosition = true;
            } else kRotationMotor.setControl(kVoltageOut.withOutput(zeroesCounterClockwise ? 1.5 : -1.5));

            if (!hasZeroedPosition) return;
        }


        if (!overrideAimTarget) aimTarget = getTargetFromPosition();

        double angleToTarget = getRotationToTarget(aimTarget);
        double minRotationBound = Math.min(maxRotationClockwise, maxRotationCounterclockwise);
        double maxRotationBound = Math.max(maxRotationClockwise, maxRotationCounterclockwise);
        canAim = !aimTarget.equals(AimTarget.NONE) && angleToTarget >= minRotationBound && angleToTarget <= maxRotationBound;

        double commandedRotation = MathUtil.clamp(angleToTarget, minRotationBound, maxRotationBound);
        setRotationDegrees(commandedRotation);

        double[] flywheelSpeeds = new double[] {calculateFlywheelSpeeds(),calculateFlywheelSpeeds()};
        
        targetBottomFlywheelSpeed = MathUtil.clamp(flywheelSpeeds[0], 0d, 100d);
        targetTopFlywheelSpeed = MathUtil.clamp(flywheelSpeeds[1], 0d, 100d);
        
        if (isIndexingSupplier.getAsBoolean()) {
            kBottomFlywheelMotor.setControl(kVelocityDutyCycle.withVelocity(targetBottomFlywheelSpeed));
            kTopFlywheelMotor.setControl(kVelocityDutyCycle.withVelocity(targetTopFlywheelSpeed));
    } else {
            kBottomFlywheelMotor.setControl(kVoltageOut.withOutput(Constants.Turret.kIdleFlywheelVoltage));
            kTopFlywheelMotor.setControl(kVoltageOut.withOutput(Constants.Turret.kIdleFlywheelVoltage));
        }
        SmartDashboard.putNumber(side+"RotationDegrees", getRotationDegrees());
    }

    public void zeroPosition() {
        this.hasZeroedPosition = false;
    }
}
