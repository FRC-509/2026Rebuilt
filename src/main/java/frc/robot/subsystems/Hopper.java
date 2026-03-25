package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    
    private final TalonFX kIntakeExtension = new TalonFX(Constants.IDs.kIntakeExtension, Constants.kCanivore);
    private PositionDutyCycle extensionDutyCycle = new PositionDutyCycle(0.0d);
    
    private final TalonFX kIntakeRotation = new TalonFX(Constants.IDs.kIntakeRotation, Constants.kCanivore);
    private VelocityDutyCycle intakeDutyCycle = new VelocityDutyCycle(0.0d);

    private final TalonFX kLeftIndexer = new TalonFX(Constants.IDs.kLeftKicker, Constants.kCanivore);
    private VelocityDutyCycle leftIndexerDutyCycle = new VelocityDutyCycle(0.0d);

    private final TalonFX kRightIndexer = new TalonFX(Constants.IDs.kRightKicker, Constants.kCanivore);
    private VelocityDutyCycle rightIndexerDutyCycle = new VelocityDutyCycle(0.0d);

    private VoltageOut voltageOut = new VoltageOut(0);

    private HopperState hopperState;
    private HopperState previousHopperState;
    
    private IndexerState indexerState;
    private IndexerState previousIndexerState;

    private boolean hasZeroedPosition;
    private double zeroedRotationOffset;
    private double commandedExtensionSetpoint;

    public enum HopperState {
        PASSIVE(false, 0.0d),
        EXTENDED(true, 0.0d),
        INDEXING(false, 0.0d), // depending on final geometry run intake wheels aswell 
        INTAKING(true, Constants.Hopper.kIntakingVelocity),
        INTAKING_AND_INDEXING(true, Constants.Hopper.kIntakingVelocity);

        public boolean hopperIsExtended;
        public double intakingVelocity;
        public double indexingVelocity;

        private HopperState(boolean hopperIsExtended, double intakingVelocity) {
            this.hopperIsExtended = hopperIsExtended;
            this.intakingVelocity = intakingVelocity;
        }
    }

    public String hopperStateString(HopperState state) {
        switch (state) {
            case PASSIVE:
                return "Passive";
            case INDEXING:
                return "Indexing";
            case INTAKING:
                return "Intaking";
            case INTAKING_AND_INDEXING:
                return "Intaking and Indexing";
        
            default:
                return "None somehow";
        }
    }

    public enum IndexerState {
        PASSIVE(false, false),
        LEFT(true,false),
        RIGHT(false,true),
        BOTH(true,true),
        REVERSE(true,true);

        public boolean leftTurret;
        public boolean rightTurret;

        private IndexerState(boolean leftTurret, boolean rightTurret) {
            this.leftTurret = leftTurret;
            this.rightTurret = rightTurret;
        }
    }

    public String indexerStateString(IndexerState state) {
        switch (state) {
            case PASSIVE:
                return "Passive";
            case LEFT:
                return "Left";
            case RIGHT:
                return "Right";
            case BOTH:
                return "Both";
        
            default:
                return "None somehow";
        }
    }

    public Hopper() {
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        
		extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		extensionConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		extensionConfig.ClosedLoopGeneral.ContinuousWrap = false;

		extensionConfig.Slot0.kP = Constants.PIDConstants.Hopper.kExtensionP;
		extensionConfig.Slot0.kI = Constants.PIDConstants.Hopper.kExtensionI;
		extensionConfig.Slot0.kD = Constants.PIDConstants.Hopper.kExtensionD;

		extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		extensionConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIntakeExtensionSupply;
        extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        extensionConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIntakeExtensionStator;

        extensionConfig.Voltage.PeakForwardVoltage = 6;
        extensionConfig.Voltage.PeakReverseVoltage = -6;

        kIntakeExtension.getConfigurator().apply(extensionConfig);


        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		intakeConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		intakeConfig.ClosedLoopGeneral.ContinuousWrap = false;

		intakeConfig.Slot0.kP = Constants.PIDConstants.Hopper.kIntakeP;
		intakeConfig.Slot0.kI = Constants.PIDConstants.Hopper.kIntakeI;
		intakeConfig.Slot0.kD = Constants.PIDConstants.Hopper.kIntakeD;

		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIntakeSupply; 
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIntakeStator;

        kIntakeRotation.getConfigurator().apply(intakeConfig);

        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
		indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		indexerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		indexerConfig.ClosedLoopGeneral.ContinuousWrap = false;

		indexerConfig.Slot0.kP = Constants.PIDConstants.Hopper.kIndexerP;
		indexerConfig.Slot0.kI = Constants.PIDConstants.Hopper.kIndexerI;
		indexerConfig.Slot0.kD = Constants.PIDConstants.Hopper.kIndexerD;

		indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		indexerConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIndexerSupply; 
        indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIndexerStator;

        kRightIndexer.getConfigurator().apply(indexerConfig);
        indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kLeftIndexer.getConfigurator().apply(indexerConfig);

        this.hopperState = HopperState.PASSIVE;
        this.previousHopperState = HopperState.PASSIVE;

        this.indexerState = IndexerState.PASSIVE;
        this.previousIndexerState = IndexerState.PASSIVE;

        this.hasZeroedPosition = false;
        this.zeroedRotationOffset = this.kIntakeExtension.getPosition().getValueAsDouble(); // temp to assume zeroed on initialize
        this.commandedExtensionSetpoint = zeroedRotationOffset;
    }

    public void setHopperState(HopperState newHopperState, IndexerState newIndexerState) {
        this.previousHopperState = this.hopperState;
        this.hopperState = newHopperState;

        this.previousIndexerState = this.indexerState;
        this.indexerState = newIndexerState;
    }

    public boolean isIndexing() {
        return hopperState == HopperState.INDEXING || hopperState == HopperState.INTAKING_AND_INDEXING;
    }

    private double clampExtensionPosition(double mechanismPosition) {
        double minPosition = zeroedRotationOffset + Constants.Hopper.kMinExtensionPosition;
        double maxPosition = zeroedRotationOffset + Constants.Hopper.kMaxExtensionPosition;
        return Math.max(minPosition, Math.min(maxPosition, mechanismPosition));
    }

    private double getExtensionSetpoint(boolean extended) {
        double relativeSetpoint = extended
            ? Constants.Hopper.kIntakeExtension
            : Constants.Hopper.kRetractedExtensionOffset;
        return clampExtensionPosition(zeroedRotationOffset + relativeSetpoint);
    }

    private double getCurrentExtensionPosition() {
        return kIntakeExtension.getPosition().getValueAsDouble() - zeroedRotationOffset;
    }

    private double getDesiredIntakeVelocity() {
        if (getCurrentExtensionPosition() < (Constants.Hopper.kMaxExtensionPosition * 0.65)) return 0.0;
        if (hopperState == HopperState.EXTENDED
            || hopperState == HopperState.INTAKING
            || hopperState == HopperState.INTAKING_AND_INDEXING) 
                return hopperState.intakingVelocity;
        return Constants.Hopper.kIntakingVelocity;
    }
    
    @Override
    public void periodic() {
        // zeroing functionality to move until you hit minimum hardstop
        if (!hasZeroedPosition) {
            if (Math.abs(kIntakeExtension.getTorqueCurrent().getValueAsDouble()) > 25) {
                zeroedRotationOffset = kIntakeExtension.getPosition().getValueAsDouble();
                commandedExtensionSetpoint = clampExtensionPosition(zeroedRotationOffset);
                kIntakeExtension.setControl(extensionDutyCycle.withPosition(commandedExtensionSetpoint));
                hasZeroedPosition = true;
            } else {
                kIntakeExtension.setControl(voltageOut.withOutput(-2));
            }

            if (!hasZeroedPosition) return;
        }

        kIntakeRotation.setControl(intakeDutyCycle.withVelocity(getDesiredIntakeVelocity()));

        if (!hopperState.hopperIsExtended
                && commandedExtensionSetpoint <= getExtensionSetpoint(false)
                && Math.abs(kIntakeExtension.getTorqueCurrent().getValueAsDouble()) >= Constants.Hopper.kRetractionResistanceTorqueThreshold) {
            commandedExtensionSetpoint = clampExtensionPosition(
                kIntakeExtension.getPosition().getValueAsDouble() + Constants.Hopper.kRetractionResistanceHoldOffset);
            kIntakeExtension.setControl(extensionDutyCycle.withPosition(commandedExtensionSetpoint));
        }

        if (!hopperState.equals(previousHopperState)) { // only change instruction on state change, not every 20ms
            if (hopperState.hopperIsExtended != previousHopperState.hopperIsExtended) { //TODO: change to actual conversion
                commandedExtensionSetpoint = getExtensionSetpoint(hopperState.hopperIsExtended);
                kIntakeExtension.setControl(extensionDutyCycle.withPosition(commandedExtensionSetpoint));
            }
        }

        if (!indexerState.equals(previousIndexerState)) { // only change instruction on state change, not every 20ms
            double isReverse = indexerState.equals(IndexerState.REVERSE) ? -1 : 1;
            if (indexerState.leftTurret != previousIndexerState.leftTurret)
                kLeftIndexer.setControl(leftIndexerDutyCycle.withVelocity(indexerState.leftTurret ? Constants.Hopper.kIndexingVelocity * isReverse : 0));
            if (indexerState.rightTurret != previousIndexerState.rightTurret)
                kRightIndexer.setControl(rightIndexerDutyCycle.withVelocity(indexerState.rightTurret ? Constants.Hopper.kIndexingVelocity  * isReverse : 0));
        }
    }

    public double getIntakeExtensionMeters() {
        double currentExtensionPosition = kIntakeExtension.getPosition().getValueAsDouble() - zeroedRotationOffset;
        double extensionRange = Constants.Hopper.kMaxExtensionPosition - Constants.Hopper.kRetractedExtensionOffset;
        if (extensionRange <= 0.0) {
            return 0.0;
        }

        double normalizedExtension = (currentExtensionPosition - Constants.Hopper.kRetractedExtensionOffset) / extensionRange;
        normalizedExtension = Math.max(0.0, Math.min(1.0, normalizedExtension));
        return normalizedExtension * Constants.Hopper.kIntakeFullExtensionMeters;
    }

    public void zeroPosition() {
        this.hasZeroedPosition = false;
    }

    public void logZero() {
    }
}
