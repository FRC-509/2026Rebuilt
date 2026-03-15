package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChoreoAuto;
import frc.robot.commands.ShootPreloadAuto;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vortex;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;

public class CenterAndDepot extends SequentialCommandGroup {
   public CenterAndDepot(SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex, Hopper hopper, Turret leftTurret, Turret rightTurret) {

        addCommands(
            new ChoreoAuto("CenterAndDepot1", swerve, pigeon, vortex),
            Commands.waitSeconds(0.4),
            ShootPreloadAuto.shootWithPositionForTime(hopper, leftTurret, rightTurret, 
                new Translation2d(
                    1.6657766103744507, 
                    2.1731061935424805),
                    5),
            Commands.runOnce(() -> hopper.setHopperState(HopperState.INTAKING, IndexerState.PASSIVE), hopper),
            
            new ChoreoAuto("CenterAndDepot2", swerve, pigeon, vortex),
            ShootPreloadAuto.shootWithPositionForTime(hopper, leftTurret, rightTurret, 
                new Translation2d(
                    1.6657766103744507,
                    1.38555109500885), 
                    5),
            Commands.runOnce(() -> hopper.setHopperState(HopperState.INTAKING, IndexerState.PASSIVE), hopper),
            
            new ChoreoAuto("CenterAndDepot3", swerve, pigeon, vortex),
            Commands.waitSeconds(0.4),
            ShootPreloadAuto.shootWithPositionForTime(hopper, leftTurret, rightTurret, 
                new Translation2d(
                    0.59184, 
                    0.6158950328826904),
                    5),
            Commands.runOnce(() -> hopper.setHopperState(HopperState.PASSIVE, IndexerState.PASSIVE), hopper)
        );
    }
}
