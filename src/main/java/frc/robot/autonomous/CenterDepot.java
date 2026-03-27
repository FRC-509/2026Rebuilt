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

public class CenterDepot extends ParallelCommandGroup{
    public CenterDepot(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        ChoreoAuto path = new ChoreoAuto("CenterDepot", swerve, pigeon, vortex);
        addCommands(
            Commands.parallel(
                path,
                ChoreoAuto.StageHopper(hopper, leftTurret, rightTurret,
                    path.new ChoreoStage(1.36, HopperState.INTAKING, IndexerState.PASSIVE),
                    path.new ChoreoStage(4.12, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(7, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(9, HopperState.INTAKING_AND_INDEXING, IndexerState.REVERSE),
                    path.new ChoreoStage(11, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(13, HopperState.INDEXING, IndexerState.BOTH))
            )
        );
    }
}
