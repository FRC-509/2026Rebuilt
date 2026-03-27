package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.choreo.ChoreoTrajectory;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vortex;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.IndexerState;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;

public class ChoreoAuto extends SequentialCommandGroup {
    public ChoreoAuto(String trajectoryName, SwerveDrive swerve, PigeonWrapper pigeon, Vortex vortex) {
        ChoreoTrajectory trajectory = ChoreoTrajectory.load(trajectoryName);

        addCommands(
            Commands.runOnce(() -> {
                if (trajectory.isEmpty()) {
                    return;
                }
                swerve.setEstimatedPoseSupplier(vortex::getEstimatedPose);
                pigeon.setYaw(trajectory.getInitialPose().getRotation().getDegrees());
                swerve.resetOdometry(trajectory.getInitialPose());
                vortex.resetEstimatedPose(trajectory.getInitialPose());
                swerve.setTargetHeading(trajectory.getInitialPose().getRotation().getDegrees());
            }, swerve),
            new FollowChoreoTrajectory(trajectory, swerve, vortex)
                .finallyDo(() -> swerve.setEstimatedPoseSupplier(vortex::getEstimatedPose)));
    }

    public static Command StageHopper(Hopper hopper, Turret leftTurret, Turret rightTurret, ChoreoStage... stages) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        double deltaT = 0.0d;
        for (ChoreoStage stage : stages) {
            sequence.addCommands(
                Commands.waitSeconds(stage.timestamp - deltaT),
                stage.waitForTurretReady
                    ? Commands.waitUntil(() -> leftTurret.isAbleToShoot() || rightTurret.isAbleToShoot())
                    : Commands.none(),
                Commands.runOnce(() -> hopper.setHopperState(stage.hopperState, stage.indexerState), hopper)         
            );
            deltaT = stage.timestamp;
        }
        return sequence;
    }

    public class ChoreoStage {
        public HopperState hopperState;
        public IndexerState indexerState;
        public double timestamp;
        public boolean waitForTurretReady;

        public ChoreoStage(double timestamp, HopperState hopperState, IndexerState indexerState){
            this(timestamp, hopperState, indexerState, false);
        }

        public ChoreoStage(double timestamp, HopperState hopperState, IndexerState indexerState, boolean waitForTurretReady){
            this.hopperState = hopperState;
            this.indexerState = indexerState;
            this.timestamp = timestamp;
            this.waitForTurretReady = waitForTurretReady;
        }
    }
}
