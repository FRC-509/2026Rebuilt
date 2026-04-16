package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ChoreoAuto;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;
import frc.robot.subsystems.Turret.AimTarget;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vortex;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;

public class RightDouble extends ParallelCommandGroup {
    public RightDouble(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        ChoreoAuto path = new ChoreoAuto("RightDouble", swerve, pigeon, vortex);
        addCommands(
            Commands.parallel(
                path,
                ChoreoAuto.StageHopper(hopper, leftTurret, rightTurret,
                    path.new ChoreoStage(0.8, HopperState.INTAKING, IndexerState.PASSIVE),
                    // first shoot
                    path.new ChoreoStage(3.0, AimTarget.HUB, 4),
                    path.new ChoreoStage(4.72, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH, true),
                    path.new ChoreoStage(5.6, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(7.7, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(8.4, HopperState.INDEXING, IndexerState.BOTH),

                    path.new ChoreoStage(9.35, HopperState.PASSIVE, IndexerState.PASSIVE),

                    // second shot
                    path.new ChoreoStage(10.63, HopperState.INTAKING, IndexerState.PASSIVE),

                    path.new ChoreoStage(15.5, AimTarget.HUB, 4),
                    path.new ChoreoStage(16.1, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(17.2, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(18.3, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(19.4, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(21, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH)
                )
            )
        );
    }
}
