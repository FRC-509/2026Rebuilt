package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.commands.ChoreoAuto;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vortex;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;

public class CenterBothMoving extends SequentialCommandGroup{
    public CenterBothMoving(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        ChoreoAuto path = new ChoreoAuto("CenterBothMoving", swerve, pigeon, vortex);
        addCommands(
            Commands.parallel(
                path,
                ChoreoAuto.StageHopper(hopper, leftTurret, rightTurret,
                    path.new ChoreoStage(0.8, HopperState.INTAKING, IndexerState.PASSIVE),

                    path.new ChoreoStage(9+1, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(11.0, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(11.7, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(12.5, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(13.7, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(14.78, HopperState.INTAKING, IndexerState.PASSIVE),

                    path.new ChoreoStage(17.5, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(19, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(21, HopperState.PASSIVE, IndexerState.BOTH)

                )
            )
        );
    }
}
