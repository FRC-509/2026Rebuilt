package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    
    private final TalonFX kIntakeExtension = new TalonFX(Constants.IDs.kIntakeExtension, Constants.kCanivore);
    private PositionDutyCycle extensionDutyCycle = new PositionDutyCycle(0.0d);
    
    // private final TalonFX kIntakeRotation = new TalonFX(Constants.IDs.kIntakeRotation, Constants.kRio);
    // private VelocityDutyCycle intakeDutyCycle = new VelocityDutyCycle(0.0d);
    
    private final TalonFX kIndexerRollers = new TalonFX(Constants.IDs.kIndexerRotation, Constants.kCanivore);
    private VelocityDutyCycle indexerRollersDutyCycle = new VelocityDutyCycle(0.0d);

    private final TalonFX kLeftIndexerRotation = new TalonFX(Constants.IDs.kIndexerRotation, Constants.kCanivore);
    private VelocityDutyCycle leftIndexerDutyCycle = new VelocityDutyCycle(0.0d);

    private final TalonFX kRightIndexerRotation = new TalonFX(Constants.IDs.kIndexerRotation, Constants.kCanivore);
    private VelocityDutyCycle rightIndexerDutyCycle = new VelocityDutyCycle(0.0d);

    private VoltageOut voltageOut = new VoltageOut(0);

    private HopperState hopperState;
    private HopperState previousHopperState;
    
    private IndexerState indexerState;
    private IndexerState previousIndexerState;

    private boolean hasZeroedPosition;
    private double zeroedRotationOffset;

    public enum HopperState {
        PASSIVE(false, 0.0d),
        INDEXING(false, 0.0d), // depending on final geometry run intake wheels aswell 
        INTAKING(true, Constants.Hopper.kIntakingVelocity),
        INTAKING_AND_INDEXING(true, Constants.Hopper.kIntakingVelocity),
        OUTTAKING(true, Constants.Hopper.kOuttakingVelocity);

        public boolean hopperIsExtended;
        public double intakingVelocity;
        public double indexingVelocity;

        private HopperState(boolean hopperIsExtended, double intakingVelocity) {
            this.hopperIsExtended = hopperIsExtended;
            this.intakingVelocity = intakingVelocity;
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

    public Hopper() {
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        
		extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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


        // TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		// intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		// intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		// intakeConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		// intakeConfig.ClosedLoopGeneral.ContinuousWrap = false;

		// intakeConfig.Slot0.kP = Constants.PIDConstants.Hopper.kIntakeP;
		// intakeConfig.Slot0.kI = Constants.PIDConstants.Hopper.kIntakeI;
		// intakeConfig.Slot0.kD = Constants.PIDConstants.Hopper.kIntakeD;

		// intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		// intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIntakeSupply; 
        // intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIntakeStator;

        // kIntakeRotation.getConfigurator().apply(intakeConfig);


        TalonFXConfiguration indexerRollerConfig = new TalonFXConfiguration();
		indexerRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		indexerRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		indexerRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;
		indexerRollerConfig.ClosedLoopGeneral.ContinuousWrap = false;

		indexerRollerConfig.Slot0.kP = Constants.PIDConstants.Hopper.kIndexerRollersP;
		indexerRollerConfig.Slot0.kI = Constants.PIDConstants.Hopper.kIndexerRollersI;
		indexerRollerConfig.Slot0.kD = Constants.PIDConstants.Hopper.kIndexerRollersD;

		indexerRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		indexerRollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.kIndexerRollersSupply; 
        indexerRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerRollerConfig.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.kIndexerRollersStator;

        kIndexerRollers.getConfigurator().apply(indexerRollerConfig);


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

        this.hopperState = HopperState.PASSIVE;
        this.previousHopperState = HopperState.PASSIVE;

        this.indexerState = IndexerState.PASSIVE;
        this.previousIndexerState = IndexerState.PASSIVE;

        this.hasZeroedPosition = false;
        this.zeroedRotationOffset = this.kIntakeExtension.getPosition().getValueAsDouble(); // temp (?) to assume zeroed on initialize
    }

    public void setHopperState(HopperState newHopperState, IndexerState newIndexerState) {
        this.previousHopperState = this.hopperState;
        this.hopperState = newHopperState;

        this.previousIndexerState = this.indexerState;
        this.indexerState = newIndexerState;
    }
    
    @Override
    public void periodic() {
        // zeroing functionality to move until you hit minimum hardstop
        if (!hasZeroedPosition) {
            if (Math.abs(kIntakeExtension.getTorqueCurrent().getValueAsDouble()) > 46) {
                kIntakeExtension.setControl(voltageOut.withOutput(0));
                zeroedRotationOffset = kIntakeExtension.getPosition().getValueAsDouble();
                hasZeroedPosition = true;
            } else kIntakeExtension.setControl(voltageOut.withOutput(-2));

            if (!hasZeroedPosition) return;
        }

        if (!hopperState.equals(previousHopperState)) { // only change instruction on state change, not every 20ms
            if (hopperState.hopperIsExtended != previousHopperState.hopperIsExtended) //TODO: change to actual conversion
                kIntakeExtension.setControl(extensionDutyCycle.withPosition(hopperState.hopperIsExtended ? Constants.Hopper.kIntakeExtension - zeroedRotationOffset : 0 - zeroedRotationOffset));
            // if (hopperState.intakingVelocity != previousHopperState.intakingVelocity) kIntakeRotation.setControl(intakeDutyCycle.withVelocity(hopperState.intakingVelocity));
        }

        if (!indexerState.equals(indexerState)) { // only change instruction on state change, not every 20ms
            kIndexerRollers.setControl(indexerRollersDutyCycle.withVelocity(!indexerState.equals(IndexerState.PASSIVE) ? Constants.Hopper.kIndexerRollersVelocity : 0));
            if (indexerState.leftTurret != previousIndexerState.leftTurret)
                kLeftIndexerRotation.setControl(leftIndexerDutyCycle.withVelocity(indexerState.leftTurret ? Constants.Hopper.kIndexingVelocity : 0));
            if (indexerState.rightTurret != previousIndexerState.rightTurret) 
                kRightIndexerRotation.setControl(rightIndexerDutyCycle.withVelocity(indexerState.rightTurret ? Constants.Hopper.kIndexingVelocity : 0));
        }


        SmartDashboard.putNumber("IntakePosition", kIntakeExtension.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("IntakeStartingOffset", zeroedRotationOffset);
        SmartDashboard.putBoolean("HasZeroed", hasZeroedPosition);
        
    }

    public double getIntakeExtensionMeters() {
        return kIntakeExtension.getPosition().getValueAsDouble() * Constants.Hopper.kIntakeExtensionToMetersConversion;
    }

    public void zeroPosition() {
        this.hasZeroedPosition = false;
    }
}
