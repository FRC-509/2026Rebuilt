package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.choreo.ChoreoTrajectory;
import frc.robot.choreo.ChoreoTrajectory.Sample;
import frc.robot.subsystems.drive.SwerveDrive;

public class FollowChoreoTrajectory extends Command {
    private final ChoreoTrajectory trajectory;
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();

    public FollowChoreoTrajectory(ChoreoTrajectory trajectory, SwerveDrive swerve) {
        this.trajectory = trajectory;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        Sample sample = trajectory.sample(timer.get());
        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx,
            sample.vy,
            sample.omega,
            swerve.getYaw()));
        swerve.setTargetHeading(sample.pose().getRotation().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.setChassisSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }
}
