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

public class CenterBoth extends ParallelCommandGroup{
    public CenterBoth(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        ChoreoAuto path = new ChoreoAuto("CenterBoth", swerve, pigeon, vortex);
        addCommands(
            Commands.parallel(
                path,
                ChoreoAuto.StageHopper(hopper, leftTurret, rightTurret,
                    path.new ChoreoStage(1.36, HopperState.INTAKING, IndexerState.BOTH),
                    path.new ChoreoStage(1.7, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(2.4, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(2.7, HopperState.INTAKING, IndexerState.PASSIVE),
                    path.new ChoreoStage(3.9, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(10, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(10.8, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(11.5, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(12.7, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(15, HopperState.INDEXING, IndexerState.BOTH)
                )
            )
        );
    }
}
