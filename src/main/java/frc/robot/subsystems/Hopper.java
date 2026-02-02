package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    
    private final TalonFX kIntakeExtension = new TalonFX(Constants.IDs.kIntakeExtension);
    private PositionDutyCycle extensionDutyCycle = new PositionDutyCycle(0.0d);
    
    private final TalonFX kIntakeRotation = new TalonFX(Constants.IDs.kIntakeRotation);
    private VelocityDutyCycle intakeDutyCycle = new VelocityDutyCycle(0.0d);

    private final TalonFX kLeftIndexerRotation = new TalonFX(Constants.IDs.kIndexerRotation);
    private VelocityDutyCycle leftIndexerDutyCycle = new VelocityDutyCycle(0.0d);

    private final TalonFX kRightIndexerRotation = new TalonFX(Constants.IDs.kIndexerRotation);
    private VelocityDutyCycle rightIndexerDutyCycle = new VelocityDutyCycle(0.0d);

    private final TalonFX kIndexerWall = new TalonFX(Constants.IDs.kIndexWallRotation);
    private PositionDutyCycle indexerWallDutyCycle = new PositionDutyCycle(0.0d);

    private HopperState hopperState;
    private HopperState previousHopperState;
    
    private IndexerState indexerState;
    private IndexerState previousIndexerState;

    public enum HopperState {
        PASSIVE(false, 0.0d, IndexerWallState.PASSIVE),
        INDEXING(false, 0.0d, IndexerWallState.ACTIVE), // depending on final geometry run intake wheels aswell 
        INTAKING(true, Constants.Hopper.kIntakingVelocity, IndexerWallState.PASSIVE),
        INTAKING_AND_INDEXING(true, Constants.Hopper.kIntakingVelocity, IndexerWallState.ACTIVE), // difference between intaking_and_ 
        FEEDING(true, Constants.Hopper.kIntakingVelocity, IndexerWallState.CONSTANT), // indexing and feeding is the indexer wall state
        OUTTAKING(true, Constants.Hopper.kOuttakingVelocity, IndexerWallState.CONSTANT);

        public boolean hopperIsExtended;
        public double intakingVelocity;
        public double indexingVelocity;
        public IndexerWallState indexerWallState;

        private HopperState(boolean hopperIsExtended, double intakingVelocity, IndexerWallState indexerWallState) {
            this.hopperIsExtended = hopperIsExtended;
            this.intakingVelocity = intakingVelocity;
            this.indexerWallState = indexerWallState;
        }
    }

    public enum IndexerState {
        PASSIVE(false, false),
        LEFT(true,false),
        RIGHT(false,true),
        BOTH(true,true);

        public boolean leftTurret;
        public boolean rightTurret;

        private IndexerState(boolean leftTurret, boolean rightTurret) {
            this.leftTurret = leftTurret;
            this.rightTurret = rightTurret;
        }
    }

    private enum IndexerWallState {
        PASSIVE(0.0),
        ACTIVE(Constants.Hopper.kIndexerWallMaximumRotation/2.0), // different implemented behavior, but fallback
        CONSTANT(Constants.Hopper.kIndexerWallMaximumRotation);

        public final double rotation;

        private IndexerWallState(double rotation) {
            this.rotation = rotation;
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
		indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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

        kLeftIndexerRotation.getConfigurator().apply(indexerConfig);
        kRightIndexerRotation.getConfigurator().apply(indexerConfig);

        
        TalonFXConfiguration indexerWallConfig = new TalonFXConfiguration();
        
		indexerWallConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		indexerWallConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		indexerWallConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		indexerWallConfig.ClosedLoopGeneral.ContinuousWrap = false;

		indexerWallConfig.Slot0.kP = Constants.PIDConstants.Hopper.kIndexerWallP;
		indexerWallConfig.Slot0.kI = Constants.PIDConstants.Hopper.kIndexerWallI;
		indexerWallConfig.Slot0.kD = Constants.PIDConstants.Hopper.kIndexerWallD;

		indexerWallConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		indexerWallConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIndexerWallSupply;
        indexerWallConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerWallConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIndexerWallStator;

        kIndexerWall.getConfigurator().apply(indexerWallConfig);

        this.hopperState = HopperState.PASSIVE;
        this.previousHopperState = HopperState.PASSIVE;

        this.indexerState = IndexerState.PASSIVE;
        this.previousIndexerState = IndexerState.PASSIVE;
    }

    public void setHopperState(HopperState newState, IndexerState indexerState) {
        this.previousHopperState = this.hopperState;
        this.hopperState = newState;
    }
    
    @Override
    public void periodic() {
        if (!hopperState.equals(previousHopperState)) { // only change instruction on state change, not every 20ms
            if (hopperState.hopperIsExtended != previousHopperState.hopperIsExtended) 
                kIntakeExtension.setControl(extensionDutyCycle.withPosition(hopperState.hopperIsExtended ? Constants.Hopper.kIntakeExtension : 0));
            if (hopperState.intakingVelocity != previousHopperState.intakingVelocity) kIntakeRotation.setControl(intakeDutyCycle.withVelocity(hopperState.intakingVelocity));
        }

        if (!indexerState.equals(indexerState)) { // only change instruction on state change, not every 20ms
            if (indexerState.leftTurret != previousIndexerState.leftTurret)
                kLeftIndexerRotation.setControl(leftIndexerDutyCycle.withVelocity(indexerState.leftTurret ? Constants.Hopper.kIndexingVelocity : 0));
            if (indexerState.rightTurret != previousIndexerState.rightTurret) 
                kRightIndexerRotation.setControl(rightIndexerDutyCycle.withVelocity(indexerState.rightTurret ? Constants.Hopper.kIndexingVelocity : 0));
        }

        if (hopperState.indexerWallState.equals(IndexerWallState.ACTIVE)){
            // fold in
            kIndexerWall.setControl(indexerWallDutyCycle.withPosition(hopperState.indexerWallState.rotation)); // TODO: replace me when sensor implemented
        } else if (!hopperState.equals(previousHopperState) && !hopperState.indexerWallState.equals(previousHopperState.indexerWallState))
            kIndexerWall.setControl(indexerWallDutyCycle.withPosition(hopperState.indexerWallState.rotation));
    }
}
