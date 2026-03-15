package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.choreo.ChoreoTrajectory;
import frc.robot.subsystems.Vortex;
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
                swerve.setEstimatedPoseSupplier(swerve::getRawOdometeryPose);
                pigeon.setYaw(trajectory.getInitialPose().getRotation().getDegrees());
                swerve.resetOdometry(trajectory.getInitialPose());
                vortex.resetEstimatedPose(trajectory.getInitialPose());
                swerve.setTargetHeading(trajectory.getInitialPose().getRotation().getDegrees());
            }, swerve),
            new FollowChoreoTrajectory(trajectory, swerve, vortex));
    }
}
