package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;
import frc.robot.subsystems.Turret;

public final class ShootPreloadAuto {
    private static final double kShootPulseIntakeSeconds = 0.9;
    private static final double kShootPulseIndexSeconds = 0.9;

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

    public static Command shootWithPositionForTime(
            Hopper hopper,
            Turret leftTurret,
            Turret rightTurret,
            Translation2d robotPosition,
            double seconds) {
        Command holdPositionOverride = Commands.startEnd(
            () -> {
                leftTurret.setOverridePositionEstimate(robotPosition);
                rightTurret.setOverridePositionEstimate(robotPosition);
            },
            () -> {
                leftTurret.clearOverridePositionEstimate();
                rightTurret.clearOverridePositionEstimate();
            },
            leftTurret,
            rightTurret);

        Command pulseShotFeed = Commands.repeatingSequence(
            Commands.runOnce(
                () -> hopper.setHopperState(HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                hopper),
            Commands.waitSeconds(kShootPulseIntakeSeconds),
            Commands.runOnce(
                () -> hopper.setHopperState(HopperState.INDEXING, IndexerState.BOTH),
                hopper),
            Commands.waitSeconds(kShootPulseIndexSeconds));

        return Commands.deadline(
            Commands.waitSeconds(seconds),
            holdPositionOverride,
            pulseShotFeed)
            .finallyDo(() -> hopper.setHopperState(HopperState.PASSIVE, IndexerState.PASSIVE));
    }
}
