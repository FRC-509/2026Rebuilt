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

    private final TalonFX kIndexerRotation = new TalonFX(Constants.IDs.kIndexerRotation);
    private VelocityDutyCycle indexerDutyCycle = new VelocityDutyCycle(0.0d);

    private final TalonFX kIndexerWall = new TalonFX(Constants.IDs.kIndexWallRotation);
    private PositionDutyCycle indexerWallDutyCycle = new PositionDutyCycle(0.0d);

    public HopperState state;
    public HopperState previousState;

    public enum HopperState {
        PASSIVE(false, 0.0d, 0.0d, IndexerWallState.PASSIVE),
        INDEXING(false, 0.0d, Constants.Hopper.kIndexingVelocity, IndexerWallState.ACTIVE), // depending on final geometry run intake wheels aswell 
        INTAKING(true, Constants.Hopper.kIntakingVelocity, Constants.Hopper.kIndexingVelocity, IndexerWallState.PASSIVE),
        OUTTAKING(true, Constants.Hopper.kOuttakingVelocity, -Constants.Hopper.kIndexingVelocity, IndexerWallState.CONSTANT);

        public boolean hopperIsExtended;
        public double intakingVelocity;
        public double indexingVelocity;
        public IndexerWallState indexerWallState;

        private HopperState(boolean hopperIsExtended, double intakingVelocity, double indexingVelocity, IndexerWallState indexerWallState) {
            this.hopperIsExtended = hopperIsExtended;
            this.intakingVelocity = intakingVelocity;
            this.indexingVelocity = indexingVelocity;
            this.indexerWallState = indexerWallState;
        }
    }

    private enum IndexerWallState {
        PASSIVE(0.0),
        ACTIVE(Constants.Hopper.kIndexerWallMaximumRotation/2.0), // different implemented behavior, but fallback
        CONSTANT(Constants.Hopper.kIndexerWallMaximumRotation);

        public double rotation;

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

        kIndexerRotation.getConfigurator().apply(indexerConfig);

        
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

        this.state = HopperState.PASSIVE;
        this.previousState = HopperState.PASSIVE;
    }

    public void setHopperState(HopperState newState) {
        this.previousState = this.state;
        this.state = newState;
    }
    
    @Override
    public void periodic() {
        if (!state.equals(previousState)) { // only change instruction on state change, not every 20ms
            if (state.hopperIsExtended != previousState.hopperIsExtended) 
                kIntakeExtension.setControl(extensionDutyCycle.withPosition(state.hopperIsExtended ? Constants.Hopper.kIntakeExtension : 0));
            if (state.intakingVelocity != previousState.intakingVelocity) kIntakeRotation.setControl(intakeDutyCycle.withVelocity(state.intakingVelocity));
            if (state.indexingVelocity != previousState.indexingVelocity) kIndexerRotation.setControl(indexerDutyCycle.withVelocity(state.indexingVelocity));
        }

        if (state.indexerWallState.equals(IndexerWallState.ACTIVE)){
            // fold in
            kIndexerWall.setControl(indexerWallDutyCycle.withPosition(state.indexerWallState.rotation)); // TODO: replace me when sensor implemented
        } else if (!state.equals(previousState) && !state.indexerWallState.equals(previousState.indexerWallState))
            kIndexerWall.setControl(indexerWallDutyCycle.withPosition(state.indexerWallState.rotation));
    }
}
