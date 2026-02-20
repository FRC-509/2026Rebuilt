package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Vortex {

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDrive swerveDrive;
    private DoubleSupplier intakeExtension;

    public Vortex(SwerveDrive swerve, Pose2d initialPose, DoubleSupplier intakeExtension) {
        this.poseEstimator = new SwerveDrivePoseEstimator(
            swerve.kinematics, 
            swerve.getYaw(),
            swerve.getModulePositions(), 
            initialPose);

        this.swerveDrive = swerve;
        this.intakeExtension = intakeExtension;

        poseEstimator.addVisionMeasurement(initialPose, Timer.getFPGATimestamp());
        poseEstimator.addVisionMeasurement(initialPose, 0, null);

        
        LimelightHelpers.setCameraPose_RobotSpace( //TODO: get real values
            Constants.Vortex.kLimelightName,
            0,
            0, 
            0,
            0,
            -10, 
            -180);
        LimelightHelpers.setPipelineIndex(Constants.Vortex.kLimelightName, 0);
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
        double[] vortexEstimate = NetworkTableInstance.getDefault().getTable("VortexFront").getEntry("Position").getDoubleArray(new double[]{0,0});
        poseEstimator.addVisionMeasurement(
            new Pose2d(
                vortexEstimate[0], vortexEstimate[1],
                swerveDrive.getYaw()),
            Timer.getFPGATimestamp(),
            Constants.Vortex.kVortexMeasurementStdDevs);

        vortexEstimate = NetworkTableInstance.getDefault().getTable("VortexBack").getEntry("Position").getDoubleArray(new double[]{0,0});
        poseEstimator.addVisionMeasurement(
            new Pose2d(
                vortexEstimate[0], vortexEstimate[1],
                swerveDrive.getYaw()),
            Timer.getFPGATimestamp(),
            Constants.Vortex.kVortexMeasurementStdDevs);

        
        LimelightHelpers.SetRobotOrientation(Constants.Vortex.kLimelightName, swerveDrive.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        PoseEstimate mt2 = DriverStation.getAlliance().get().equals(Alliance.Red)
            ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(Constants.Vortex.kLimelightName) // TODO: double check which to use, conditional needed?
            : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vortex.kLimelightName);
   

        // if our angular velocity is greater than 360 degrees per second or theres no tags, ignore vision updates
        if(!(Math.abs(swerveDrive.getAngularVelocity()) > 360 // TODO: tune w threshold
                || mt2.tagCount == 0 
                || LimelightHelpers.getCurrentPipelineIndex(Constants.Vortex.kLimelightName) != 0)) {

            mt2.pose = new Pose2d( // TODO: double check sign of vector
                mt2.pose.getX() + intakeExtension.getAsDouble() * Math.cos(swerveDrive.getYaw().getRadians()), // account for extending intake by 
                mt2.pose.getY() + intakeExtension.getAsDouble() * -Math.sin(swerveDrive.getYaw().getRadians()), // adding rotated extension vector
                swerveDrive.getYaw());

            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds,
                Constants.Vortex.kLimelightMeasurementStdDevs);
        }

        return poseEstimator.update(swerveDrive.getYaw(), swerveDrive.getModulePositions()).getTranslation();
    }

}
