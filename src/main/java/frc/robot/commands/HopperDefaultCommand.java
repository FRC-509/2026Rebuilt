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
    private BooleanSupplier leftIndexingSupplier;
    private BooleanSupplier rightIndexingSupplier; 
    private BooleanSupplier outtakingSupplier;

    public HopperDefaultCommand(
            Hopper hopper, 
            BooleanSupplier intakeSupplier, 
            BooleanSupplier indexingSupplier,
            BooleanSupplier leftIndexingSupplier,
            BooleanSupplier rightIndexingSupplier, 
            BooleanSupplier outtakingSupplier) {
        this.hopper = hopper;

        this.intakeSupplier = intakeSupplier;
        this.indexingSupplier = indexingSupplier;
        this.leftIndexingSupplier = leftIndexingSupplier;
        this.rightIndexingSupplier = rightIndexingSupplier;
        this.outtakingSupplier = outtakingSupplier;

        addRequirements(hopper);
    }

    public HopperDefaultCommand( // for use in auto
            Hopper hopper, 
            boolean isIntaking, 
            boolean isIndexing, 
            boolean isFeeding, 
            boolean isOuttaking,
            BooleanSupplier leftTurretCanShootSupplier,
            BooleanSupplier rightTurretCanShootSupplier) {
        this.hopper = hopper;

        this.intakeSupplier = () -> isIntaking;
        this.indexingSupplier = () -> isIndexing;
        this.outtakingSupplier = () -> isOuttaking;
        this.leftIndexingSupplier = leftTurretCanShootSupplier;
        this.rightIndexingSupplier = rightTurretCanShootSupplier;

        addRequirements(hopper);
    }

    @Override
    public void execute() {
        if (intakeSupplier.getAsBoolean()) {
            if (indexingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.INTAKING_AND_INDEXING, getDesiredIndexerState());
            else hopper.setHopperState(HopperState.INTAKING, getDesiredIndexerState());
        } else if (indexingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.INDEXING, getDesiredIndexerState());
        else if (outtakingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.OUTTAKING, IndexerState.PASSIVE);
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
