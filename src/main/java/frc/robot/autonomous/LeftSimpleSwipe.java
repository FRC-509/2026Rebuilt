package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.commands.ChoreoAuto;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vortex;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;

public class LeftSimpleSwipe extends ParallelCommandGroup{
    public LeftSimpleSwipe(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        ChoreoAuto path = new ChoreoAuto("LeftSimpleSwipe", swerve, pigeon, vortex);
        addCommands(
            Commands.parallel(
                path,
                ChoreoAuto.StageHopper(hopper, leftTurret, rightTurret,
                    path.new ChoreoStage(0.8, HopperState.INTAKING, IndexerState.PASSIVE),
                    // first shoot
                    path.new ChoreoStage(6.22, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(7.3, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(7.95, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(8.5, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(9, HopperState.INTAKING, IndexerState.PASSIVE),
                    // pickup and shoot at outpost
                    path.new ChoreoStage(14.4, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(15.1, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(15.8, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH)
                )
            )
        );
    }
}
