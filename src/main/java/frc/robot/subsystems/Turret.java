package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Translation2dSupplier;

public class Turret extends SubsystemBase {
    
    private final TalonFX kRotationMotor;
    private final TalonFX kTopFlywheelMotor;
    private final TalonFX kBottomFlywheelMotor;

    private final VelocityVoltage kVelocityVoltage = new VelocityVoltage(0.0d);
    private final PositionDutyCycle kPositionDutyCycle = new PositionDutyCycle(0.0d);

    private final Translation3d offsetTranslation;
    private final double maxRotationClockwise;
    private final double maxRotationCounterclockwise;

    private final Translation2dSupplier positionEstimate;
    private final DoubleSupplier robotYawDegreesSupplier;

    private AimTarget aimTarget;
    private boolean canShoot;

    public Turret(
            int rotationMotorId, 
            int topFlywheelMotorId, 
            int bottomFlywheelMotorId,
            Translation3d offsetTranslation,
            double maxRotationClockwise,
            double maxRotationCounterclockwise,
            Translation2dSupplier positionEstimate,
            DoubleSupplier robotYawDegreesSupplier) {
        this.kRotationMotor = new TalonFX(rotationMotorId);
        this.kTopFlywheelMotor = new TalonFX(topFlywheelMotorId);
        this.kBottomFlywheelMotor = new TalonFX(bottomFlywheelMotorId);

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
        this.offsetTranslation = offsetTranslation;
        this.maxRotationClockwise = maxRotationClockwise;
        this.maxRotationCounterclockwise = maxRotationCounterclockwise;
        this.robotYawDegreesSupplier = robotYawDegreesSupplier;

        this.aimTarget = AimTarget.HOPPER;
        this.canShoot = false;
    }
 
    public enum AimTarget {

        HOPPER(new Translation3d(), new Translation3d());

        Translation3d bluePosition;
        Translation3d redPosition;

        private AimTarget(Translation3d bluePosition, Translation3d redPosition) {
            this.bluePosition = bluePosition;
            this.redPosition = redPosition;
        }

        public Translation3d getBasedOnAlliance() {
            return DriverStation.getAlliance().get().equals(Alliance.Red) ? redPosition : bluePosition;
        }
    }

    public void setRotation(double degrees) {
        kRotationMotor.setControl(kPositionDutyCycle.withPosition(degrees));
    }

    public double getRotationToTarget(AimTarget targetPosition) {
        Translation2d turretGlobal = positionEstimate.getAsTranslation2d().plus(offsetTranslation.toTranslation2d());
        Translation2d targetTurretRelative = targetPosition.getBasedOnAlliance().toTranslation2d().minus(turretGlobal);

        double angle = Math.atan2(targetTurretRelative.getY(), targetTurretRelative.getX()) - robotYawDegreesSupplier.getAsDouble(); // TODO: confirm minus
        return angle;
    }

    public boolean isAbleToShoot() {
        return canShoot;
    }

    @Override
    public void periodic() {
        double angleToTarget = getRotationToTarget(aimTarget);
        this.canShoot = angleToTarget <= maxRotationClockwise || angleToTarget >= maxRotationCounterclockwise; // TODO: double check which direction is min and max        
        setRotation(canShoot ? angleToTarget : 0);
    }
}
