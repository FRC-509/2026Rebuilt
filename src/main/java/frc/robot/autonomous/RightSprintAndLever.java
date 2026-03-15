package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ChoreoAuto;
import frc.robot.commands.ShootPreloadAuto;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vortex;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PigeonWrapper;

public class RightSprintAndLever extends ParallelCommandGroup {
   public RightSprintAndLever(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {

        addCommands(
            new ChoreoAuto("RightSprintAndLever1", swerve, pigeon, vortex),
            Commands.sequence(
                Commands.waitSeconds(0.9),
                Commands.runOnce(() -> hopper.setHopperState(HopperState.INTAKING, IndexerState.PASSIVE), hopper),
                Commands.waitSeconds(3.14 - 0.9),
                Commands.runOnce(() -> hopper.setHopperState(HopperState.PASSIVE, IndexerState.PASSIVE), hopper),
                Commands.waitSeconds(5.2 - 3.14 - 0.9)
            ),
            ShootPreloadAuto.shootWithPositionForTime(hopper, leftTurret, rightTurret,
                new Translation2d(
                    2.5181660652160645, 
                    2.6205806732177734), 
                    5),
            new ChoreoAuto("RightSprintAndLever2", swerve, pigeon, vortex),
            ShootPreloadAuto.shootWithPositionForTime(hopper, leftTurret, rightTurret,
                new Translation2d(
                    0.5560398697853088,
                    0.598),
                    5)
        );
    }
}
