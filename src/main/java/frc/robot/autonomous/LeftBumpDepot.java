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

public class LeftBumpDepot extends ParallelCommandGroup{
    public LeftBumpDepot(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        ChoreoAuto path = new ChoreoAuto("LeftBumpDepot", swerve, pigeon, vortex);
        addCommands(
            Commands.parallel(
                path,
                ChoreoAuto.StageHopper(hopper, leftTurret, rightTurret,
                    path.new ChoreoStage(3, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(5, HopperState.PASSIVE, IndexerState.PASSIVE),
                    path.new ChoreoStage(7.72, HopperState.INTAKING, IndexerState.PASSIVE),
                    path.new ChoreoStage(8, HopperState.EXTENDED, IndexerState.PASSIVE),
                    path.new ChoreoStage(11.36, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(12.8, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(14.4, HopperState.INTAKING, IndexerState.BOTH)
                )
            )
        );
    }
}
