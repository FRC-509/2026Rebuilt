package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

public class Vortex {

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDrive swerveDrive;

    public Vortex(SwerveDrive swerve, Pose2d initialPose) {
        this.poseEstimator = new SwerveDrivePoseEstimator(
            swerve.kinematics, 
            swerve.getYaw(),
            swerve.getModulePositions(), 
            initialPose);

        this.swerveDrive = swerve;

		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 99999)); // lower confidence?
        poseEstimator.addVisionMeasurement(initialPose, Timer.getFPGATimestamp());
    }

    public Translation2d getEstimatedGlobalPosition() {
        return updatePositionEstimate();
    }

    public Translation2d getEstimatedAlliancePosition() {
        /* Field orientation
        * ^
	    * | X+
	    * ----> Y-
        */
        return DriverStation.getAlliance().get().equals(Alliance.Red) //TODO: confirm blue alliance has 0,0
            ? new Translation2d(Constants.Field.kFullFieldLength,Constants.Field.kFieldWidth).minus(getEstimatedGlobalPosition())
            : getEstimatedGlobalPosition();
    }

    public Translation2d updatePositionEstimate() {
        double[] vortexEstimate = NetworkTableInstance.getDefault().getTable("Vortex").getEntry("Position").getDoubleArray(new double[]{0,0});
        poseEstimator.addVisionMeasurement(
            new Pose2d(
                vortexEstimate[0], vortexEstimate[1],
                swerveDrive.getYaw()),
            Timer.getFPGATimestamp());

        return poseEstimator.update(swerveDrive.getYaw(), swerveDrive.getModulePositions()).getTranslation();
    }

}
