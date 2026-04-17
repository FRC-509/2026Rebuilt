package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;

public class HopperDefaultCommand extends Command {
    
    private final Hopper hopper;

    private BooleanSupplier intakeSupplier;
    private BooleanSupplier outtakeSupplier;
    private BooleanSupplier keepIntakeExtendedSupplier;
    private BooleanSupplier indexingSupplier;
    private BooleanSupplier reverseIndexingSupplier;
    private BooleanSupplier prefireSupplier;
    private BooleanSupplier manualFeedOverrideSupplier;
    private BooleanSupplier leftFeedIntentSupplier;
    private BooleanSupplier rightFeedIntentSupplier;
    private BooleanSupplier leftFeedReadySupplier;
    private BooleanSupplier rightFeedReadySupplier;

    public HopperDefaultCommand(
            Hopper hopper,
            BooleanSupplier intakeSupplier,
            BooleanSupplier outtakeSupplier,
            BooleanSupplier keepIntakeExtendedSupplier,
            BooleanSupplier indexingSupplier,
            BooleanSupplier reverseIndexingSupplier,
            BooleanSupplier prefireSupplier,
            BooleanSupplier manualFeedOverrideSupplier,
            BooleanSupplier leftFeedIntentSupplier,
            BooleanSupplier rightFeedIntentSupplier,
            BooleanSupplier leftFeedReadySupplier,
            BooleanSupplier rightFeedReadySupplier) {
        this.hopper = hopper;

        this.intakeSupplier = intakeSupplier;
        this.outtakeSupplier = outtakeSupplier;
        this.keepIntakeExtendedSupplier = keepIntakeExtendedSupplier;
        this.indexingSupplier = indexingSupplier;
        this.reverseIndexingSupplier = reverseIndexingSupplier;
        this.prefireSupplier = prefireSupplier;
        this.manualFeedOverrideSupplier = manualFeedOverrideSupplier;
        this.leftFeedIntentSupplier = leftFeedIntentSupplier;
        this.rightFeedIntentSupplier = rightFeedIntentSupplier;
        this.leftFeedReadySupplier = leftFeedReadySupplier;
        this.rightFeedReadySupplier = rightFeedReadySupplier;

        addRequirements(hopper);
    }

    public HopperDefaultCommand( // for use in auto
            Hopper hopper, 
            boolean isIntaking,
            boolean isIndexing,
            boolean isPrefiring,
            boolean isFeeding,
            BooleanSupplier leftFeedReadySupplier,
            BooleanSupplier rightFeedReadySupplier) {
        this.hopper = hopper;

        this.intakeSupplier = () -> isIntaking;
        this.outtakeSupplier = () -> false;
        this.keepIntakeExtendedSupplier = () -> false;
        this.indexingSupplier = () -> isIndexing;
        this.reverseIndexingSupplier = () -> false;
        this.prefireSupplier = () -> isPrefiring;
        this.manualFeedOverrideSupplier = () -> false;
        this.leftFeedIntentSupplier = () -> isFeeding;
        this.rightFeedIntentSupplier = () -> isFeeding;
        this.leftFeedReadySupplier = leftFeedReadySupplier;
        this.rightFeedReadySupplier = rightFeedReadySupplier;

        addRequirements(hopper);
    }

    @Override
    public void execute() {
        boolean manualIndexing = indexingSupplier.getAsBoolean();
        boolean shouldIndex = manualIndexing || prefireSupplier.getAsBoolean();
        if (outtakeSupplier.getAsBoolean()) hopper.setHopperState(HopperState.OUTTAKING, IndexerState.REVERSE);
        else if (intakeSupplier.getAsBoolean()) {
            if (reverseIndexingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.INTAKING_AND_INDEXING, IndexerState.REVERSE);
            else if (shouldIndex) hopper.setHopperState(HopperState.INTAKING_AND_INDEXING, getDesiredIndexerState());
            else hopper.setHopperState(HopperState.INTAKING, IndexerState.PASSIVE);
        }
        else if (keepIntakeExtendedSupplier.getAsBoolean()) {
            if (reverseIndexingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.EXTENDED, IndexerState.REVERSE);
            else if (shouldIndex) hopper.setHopperState(HopperState.EXTENDED, getDesiredIndexerState());
            else hopper.setHopperState(HopperState.EXTENDED, IndexerState.PASSIVE);
        }
        else if (reverseIndexingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.INDEXING, IndexerState.REVERSE);
        else if (shouldIndex) hopper.setHopperState(HopperState.INDEXING, getDesiredIndexerState());
        else hopper.setHopperState(HopperState.PASSIVE, IndexerState.PASSIVE);
    }

    private IndexerState getDesiredIndexerState() {
        if (indexingSupplier.getAsBoolean()) {
            if (manualFeedOverrideSupplier.getAsBoolean()) {
                return IndexerState.BOTH;
            }

            boolean wantsFeedTargets = leftFeedIntentSupplier.getAsBoolean() || rightFeedIntentSupplier.getAsBoolean();
            boolean shouldFeedLeft = wantsFeedTargets
                ? leftFeedIntentSupplier.getAsBoolean() && leftFeedReadySupplier.getAsBoolean()
                : leftFeedReadySupplier.getAsBoolean();
            boolean shouldFeedRight = wantsFeedTargets
                ? rightFeedIntentSupplier.getAsBoolean() && rightFeedReadySupplier.getAsBoolean()
                : rightFeedReadySupplier.getAsBoolean();

            if (shouldFeedLeft && shouldFeedRight) {
                return IndexerState.BOTH;
            }
            if (shouldFeedLeft) {
                return IndexerState.LEFT;
            }
            if (shouldFeedRight) {
                return IndexerState.RIGHT;
            }
            return IndexerState.PASSIVE;
        }

        boolean shouldFeedLeft = leftFeedIntentSupplier.getAsBoolean() && leftFeedReadySupplier.getAsBoolean();
        boolean shouldFeedRight = rightFeedIntentSupplier.getAsBoolean() && rightFeedReadySupplier.getAsBoolean();

        if (shouldFeedLeft && shouldFeedRight) {
            return IndexerState.BOTH;
        }
        if (shouldFeedLeft) {
            return IndexerState.LEFT;
        }
        if (shouldFeedRight) {
            return IndexerState.RIGHT;
        }

        return IndexerState.PASSIVE;
    }

}
