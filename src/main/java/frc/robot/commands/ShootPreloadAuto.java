package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;

public final class ShootPreloadAuto {
    private ShootPreloadAuto() {}

    public static Command create(Hopper hopper) {
        return Commands.startEnd(
            () -> hopper.setHopperState(HopperState.INDEXING, IndexerState.BOTH),
            () -> hopper.setHopperState(HopperState.PASSIVE, IndexerState.PASSIVE), 
            hopper)
            .withTimeout(6.0)
            .andThen(Commands.runOnce(
                () -> hopper.setHopperState(HopperState.PASSIVE, IndexerState.PASSIVE),
                hopper));
    }
}
