package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChoreoAuto;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;

public class CenterAndDepot extends SequentialCommandGroup {
   public CenterAndDepot(SwerveDrive swerve, PigeonWrapper pigeon, Hopper hopper, Turret leftTurret, Turret rightTurret) {

        addCommands(
            Commands.waitSeconds(1),
            Commands.parallel(
                new ChoreoAuto("CenterAndDepot", swerve, pigeon),
                Commands.sequence(
                    Commands.waitSeconds(0.9),
                    Commands.runOnce(() -> hopper.setHopperState(HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH), hopper),
                    Commands.waitSeconds(2 - 0.9),
                    Commands.runOnce(() -> hopper.setHopperState(HopperState.INTAKING, IndexerState.PASSIVE), hopper),
                    Commands.waitSeconds(4.23-2-0.9),
                    // Commands.waitUntil(() -> LimelightHelpers.getTV(Constants.Vortex.kFrontLimelightName)),
                    Commands.runOnce(() -> hopper.setHopperState(HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH), hopper),
                    Commands.waitSeconds(0.9),
                    Commands.runOnce(() -> hopper.setHopperState(HopperState.INDEXING, IndexerState.BOTH), hopper),
                    Commands.waitSeconds(0.9),
                    Commands.runOnce(() -> hopper.setHopperState(HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH), hopper)
                )
            )
        );
    }
}
