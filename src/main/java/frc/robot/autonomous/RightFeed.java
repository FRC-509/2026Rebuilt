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

public class RightFeed extends ParallelCommandGroup {
    public RightFeed(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        ChoreoAuto path = new ChoreoAuto("RightFeed", swerve, pigeon, vortex);
        addCommands(
            Commands.parallel(
                path,
                ChoreoAuto.StageHopper(hopper, leftTurret, rightTurret,
                    path.new ChoreoStage(0.3, AimTarget.NEUTRALZONE_FEED_RIGHT, 2),
                    path.new ChoreoStage(0.8, HopperState.INTAKING, IndexerState.BOTH, true),
                    // first shoot
                    path.new ChoreoStage(7, AimTarget.HUB, 4),
                    path.new ChoreoStage(9.62, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH, true),
                    // pickup and shoot at outpost
                    path.new ChoreoStage(16.4, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(17.1, HopperState.INDEXING, IndexerState.BOTH),
                    path.new ChoreoStage(17.8, HopperState.INTAKING_AND_INDEXING, IndexerState.BOTH)
                )
            )
        );    }
}
