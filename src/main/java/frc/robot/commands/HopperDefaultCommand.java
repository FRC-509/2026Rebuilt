package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;

public class HopperDefaultCommand extends Command {
    
    private final Hopper hopper;

    private BooleanSupplier intakeSupplier;
    private BooleanSupplier indexingSupplier; 
    private BooleanSupplier feedingSupplier;
    private BooleanSupplier outtakingSupplier;

    public HopperDefaultCommand(
            Hopper hopper, 
            BooleanSupplier intakeSupplier, 
            BooleanSupplier indexingSupplier, 
            BooleanSupplier feedingSupplier, 
            BooleanSupplier outtakingSupplier) {
        this.hopper = hopper;

        this.intakeSupplier = intakeSupplier;
        this.indexingSupplier = indexingSupplier;
        this.feedingSupplier = feedingSupplier;
        this.outtakingSupplier = outtakingSupplier;

        addRequirements(hopper);
    }

    @Override
    public void execute() {
        if (intakeSupplier.getAsBoolean()) {
            if (indexingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.INTAKING_AND_INDEXING);
            else hopper.setHopperState(HopperState.INTAKING);
        } else if (indexingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.INDEXING);
        else if (outtakingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.OUTTAKING);
        else if (feedingSupplier.getAsBoolean()) hopper.setHopperState(HopperState.FEEDING);
        else hopper.setHopperState(HopperState.PASSIVE);
    }

}
