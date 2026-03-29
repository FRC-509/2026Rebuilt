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

public class SimpleCenterDepot extends ParallelCommandGroup{
    public SimpleCenterDepot(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        ChoreoAuto path = new ChoreoAuto("SimpleCenterDepot", swerve, pigeon, vortex);
        addCommands(
            Commands.parallel(
                path,
                ChoreoAuto.StageHopper(hopper, leftTurret, rightTurret,
                    path.new ChoreoStage(0.7, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(1.34, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(4.5, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(6.8, HopperState.INDEXING, IndexerState.BOTH)
                )
            )
        );
    }
}
