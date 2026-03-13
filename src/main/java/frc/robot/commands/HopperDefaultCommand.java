package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;

public class HopperDefaultCommand extends Command {
    
    private final Hopper hopper;

    private BooleanSupplier intakeSupplier;
    private BooleanSupplier indexingSupplier;
    private BooleanSupplier prefireSupplier;
    private BooleanSupplier leftIndexingSupplier;
    private BooleanSupplier rightIndexingSupplier; 

    public HopperDefaultCommand(
            Hopper hopper,
            BooleanSupplier intakeSupplier, 
            BooleanSupplier indexingSupplier,
            BooleanSupplier prefireSupplier,
            BooleanSupplier leftIndexingSupplier,
            BooleanSupplier rightIndexingSupplier) {
        this.hopper = hopper;

        this.intakeSupplier = intakeSupplier;
        this.indexingSupplier = indexingSupplier;
        this.prefireSupplier = prefireSupplier;
        this.leftIndexingSupplier = leftIndexingSupplier;
        this.rightIndexingSupplier = rightIndexingSupplier;

        addRequirements(hopper);
    }

    public HopperDefaultCommand( // for use in auto
            Hopper hopper, 
            boolean isIntaking,
            boolean isIndexing,
            boolean isPrefiring,
            boolean isFeeding,
            BooleanSupplier leftTurretCanShootSupplier,
            BooleanSupplier rightTurretCanShootSupplier) {
        this.hopper = hopper;

        this.intakeSupplier = () -> isIntaking;
        this.indexingSupplier = () -> isIndexing;
        this.prefireSupplier = () -> isPrefiring;
        this.leftIndexingSupplier = leftTurretCanShootSupplier;
        this.rightIndexingSupplier = rightTurretCanShootSupplier;

        addRequirements(hopper);
    }

    @Override
    public void execute() {
        boolean shouldIndex = indexingSupplier.getAsBoolean() || prefireSupplier.getAsBoolean();
        if (intakeSupplier.getAsBoolean()) {
            if (shouldIndex) hopper.setHopperState(HopperState.INTAKING_AND_INDEXING, getDesiredIndexerState());
            else hopper.setHopperState(HopperState.INTAKING, IndexerState.PASSIVE);
        } else if (shouldIndex) hopper.setHopperState(HopperState.INDEXING, getDesiredIndexerState());
        else hopper.setHopperState(HopperState.PASSIVE, IndexerState.PASSIVE);
    }

    private IndexerState getDesiredIndexerState() {
        if (leftIndexingSupplier.getAsBoolean()) {
            if (rightIndexingSupplier.getAsBoolean()) return IndexerState.BOTH;
            return IndexerState.LEFT;           
        } else if (rightIndexingSupplier.getAsBoolean()) return IndexerState.RIGHT;
        return IndexerState.PASSIVE;
    }

}
