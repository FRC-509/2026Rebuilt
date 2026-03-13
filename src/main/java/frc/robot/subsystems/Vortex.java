package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.JetsonUdpRelay;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Vortex {
    private static final double JETSON_STALE_SECONDS = 0.5;

    private static final String FRONT_JETSON_TABLE_NAME = "VortexFront";
    private static final String BACK_JETSON_TABLE_NAME = "VortexBack";
    private static final String VORTEX_TABLE_NAME = "Vortex";

    private final NetworkTable frontJetsonTable;
    private final NetworkTable backJetsonTable;
    private final NetworkTable vortexTable;

    private final JetsonUdpRelay frontJetson;
    private final JetsonUdpRelay backJetson;

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier intakeExtension;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;

    private double lastFrontJetsonMeasurementTimestamp = Double.NaN;
    private double lastBackJetsonMeasurementTimestamp = Double.NaN;
    private double lastLimelightMeasurementTimestamp = Double.NaN;

    public Vortex(SwerveDrive swerve, Pose2d initialPose, DoubleSupplier intakeExtension) {
        this.swerveDrive = swerve;
        this.intakeExtension = intakeExtension;

        JetsonUdpRelay frontJetsonRelay;
        JetsonUdpRelay backJetsonRelay;
        try {
            frontJetsonRelay = new JetsonUdpRelay(FRONT_JETSON_TABLE_NAME, 5091);
            backJetsonRelay = new JetsonUdpRelay(BACK_JETSON_TABLE_NAME, 5092);
        } catch (Exception e) {
            DriverStation.reportError("Jetson failed to initialize.", false);
            frontJetsonRelay = null;
            backJetsonRelay = null;
        }

        this.frontJetson = frontJetsonRelay;
        this.backJetson = backJetsonRelay;
        this.frontJetsonTable = NetworkTableInstance.getDefault().getTable("/Vortex/" + FRONT_JETSON_TABLE_NAME);
        this.backJetsonTable = NetworkTableInstance.getDefault().getTable("/Vortex/" + BACK_JETSON_TABLE_NAME);
        this.vortexTable = NetworkTableInstance.getDefault().getTable(VORTEX_TABLE_NAME);

        this.poseEstimator = new SwerveDrivePoseEstimator(
            swerve.kinematics,
            swerve.getYaw(),
            swerve.getModulePositions(),
            initialPose);
        this.field2d = new Field2d();

        swerve.resetOdometry(initialPose);
        swerve.setEstimatedPoseSupplier(poseEstimator::getEstimatedPosition);
        field2d.setRobotPose(initialPose);
        SmartDashboard.putData("Field", field2d);

        LimelightHelpers.setCameraPose_RobotSpace(
            Constants.Vortex.kFrontLimelightName,
            Constants.Vortex.kFrontLimelightForwardMeters,
            Constants.Vortex.kFrontLimelightSideMeters,
            Constants.Vortex.kFrontLimelightUpMeters,
            Constants.Vortex.kFrontLimelightRollDegrees,
            Constants.Vortex.kFrontLimelightPitchDegrees,
            Constants.Vortex.kFrontLimelightYawDegrees);
        LimelightHelpers.setPipelineIndex(Constants.Vortex.kFrontLimelightName, 0);
    }

    public Translation2d getEstimatedGlobalPosition() {
        return poseEstimator.getEstimatedPosition().getTranslation();
    }

    public Translation2d getEstimatedAlliancePosition() {
        return toAllianceRelative(getEstimatedGlobalPosition());
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getEstimatedAlliancePose() {
        return toAllianceRelative(getEstimatedPose());
    }

    public Translation2d updatePositionEstimate() {
        poseEstimator.update(swerveDrive.getYaw(), swerveDrive.getModulePositions());

        lastFrontJetsonMeasurementTimestamp = applyJetsonMeasurement(frontJetsonTable, lastFrontJetsonMeasurementTimestamp);
        lastBackJetsonMeasurementTimestamp = applyJetsonMeasurement(backJetsonTable, lastBackJetsonMeasurementTimestamp);
        applyFrontLimelightMeasurement();
        postVortexToNT();

        return getEstimatedGlobalPosition();
    }

    private double applyJetsonMeasurement(NetworkTable jetsonTable, double lastMeasurementTimestamp) {
        if (!hasFreshJetsonPose(jetsonTable)) {
            return lastMeasurementTimestamp;
        }

        double measurementTimestamp = jetsonTable.getEntry("last_packet_timestamp").getDouble(Double.NaN);
        if (Double.isNaN(measurementTimestamp) || measurementTimestamp <= lastMeasurementTimestamp) {
            return lastMeasurementTimestamp;
        }

        poseEstimator.addVisionMeasurement(
            getJetsonPose(jetsonTable),
            measurementTimestamp,
            Constants.Vortex.kVortexMeasurementStdDevs);
        return measurementTimestamp;
    }

    private void applyFrontLimelightMeasurement() {
        LimelightHelpers.SetRobotOrientation(
            Constants.Vortex.kFrontLimelightName,
            swerveDrive.getYaw().getDegrees(),
            0,
            0,
            0,
            0,
            0);

        PoseEstimate poseEstimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vortex.kFrontLimelightName);

        if (poseEstimate == null
                || poseEstimate.pose == null
                || Math.abs(swerveDrive.getAngularVelocity()) > 360
                || poseEstimate.tagCount == 0
                || poseEstimate.timestampSeconds <= lastLimelightMeasurementTimestamp
                || LimelightHelpers.getCurrentPipelineIndex(Constants.Vortex.kFrontLimelightName) != 0) {
            return;
        }

        poseEstimator.addVisionMeasurement(
            new Pose2d(poseEstimate.pose.getTranslation(), swerveDrive.getYaw()),
            poseEstimate.timestampSeconds,
            Constants.Vortex.kLimelightMeasurementStdDevs);
        lastLimelightMeasurementTimestamp = poseEstimate.timestampSeconds;
    }

    private boolean hasFreshJetsonPose(NetworkTable jetsonTable) {
        double lastPacketTimestamp = jetsonTable.getEntry("last_packet_timestamp").getDouble(Double.NaN);
        return !Double.isNaN(lastPacketTimestamp)
            && (Timer.getFPGATimestamp() - lastPacketTimestamp) <= JETSON_STALE_SECONDS
            && jetsonTable.getEntry("robot/has_pose").getBoolean(false);
    }

    private Pose2d getJetsonPose(NetworkTable jetsonTable) {
        return new Pose2d(
            jetsonTable.getEntry("robot/x").getDouble(0.0),
            jetsonTable.getEntry("robot/y").getDouble(0.0),
            swerveDrive.getYaw());
    }

    public Translation2d getLatestJetsonPose() {
        Translation2d sum = Translation2d.kZero;
        int measurementCount = 0;

        if (hasFreshJetsonPose(frontJetsonTable)) {
            sum = sum.plus(getJetsonPose(frontJetsonTable).getTranslation());
            measurementCount++;
        }
        if (hasFreshJetsonPose(backJetsonTable)) {
            sum = sum.plus(getJetsonPose(backJetsonTable).getTranslation());
            measurementCount++;
        }

        if (measurementCount == 0) {
            return getEstimatedAlliancePosition();
        }

        return toAllianceRelative(sum.div(measurementCount));
    }

    public void pollJetsons() {
        if (frontJetson != null) {
            frontJetson.poll();
        }
        if (backJetson != null) {
            backJetson.poll();
        }

        Translation2d estimatedPosition = updatePositionEstimate();
        Translation2d allianceEstimatedPosition = getEstimatedAlliancePosition();
        Translation2d jetsonPosition = getLatestJetsonPose();
        SmartDashboard.putNumber("JetsonX", jetsonPosition.getX());
        SmartDashboard.putNumber("JetsonY", jetsonPosition.getY());
        SmartDashboard.putNumber("EstimatorX", allianceEstimatedPosition.getX());
        SmartDashboard.putNumber("EstimatorY", allianceEstimatedPosition.getY());
        SmartDashboard.putNumber("IntakeExtensionMeters", intakeExtension.getAsDouble());
    }

    private void postVortexToNT() {
        Pose2d estimatedPose = getEstimatedAlliancePose();

        vortexTable.getEntry("EstimatedPose").setDoubleArray(new double[] {
            estimatedPose.getX(),
            estimatedPose.getY(),
            estimatedPose.getRotation().getDegrees()
        });
        vortexTable.getEntry("HasFrontJetsonPose").setBoolean(hasFreshJetsonPose(frontJetsonTable));
        vortexTable.getEntry("HasBackJetsonPose").setBoolean(hasFreshJetsonPose(backJetsonTable));
        vortexTable.getEntry("HasLimelightPose").setBoolean(!Double.isNaN(lastLimelightMeasurementTimestamp));

        field2d.setRobotPose(estimatedPose);
        field2d.getObject("front_jetson_pose").setPose(getJetsonPose(frontJetsonTable));
        field2d.getObject("back_jetson_pose").setPose(getJetsonPose(backJetsonTable));

        if (!Double.isNaN(lastLimelightMeasurementTimestamp)) {
            vortexTable.getEntry("LastLimelightTimestamp").setDouble(lastLimelightMeasurementTimestamp);
        }
    }

    private Translation2d toAllianceRelative(Translation2d globalPosition) {
        return SwerveDrive.getAlliance() == Alliance.Red
            ? new Translation2d(
                Constants.Field.kFullFieldLength - globalPosition.getX(),
                Constants.Field.kFieldWidth - globalPosition.getY())
            : globalPosition;
    }

    private Pose2d toAllianceRelative(Pose2d globalPose) {
        if (SwerveDrive.getAlliance() != Alliance.Red) {
            return globalPose;
        }

        return new Pose2d(
            toAllianceRelative(globalPose.getTranslation()),
            globalPose.getRotation());
    }

    public void closeJetsons() {
        if (frontJetson != null) {
            frontJetson.close();
        }
        if (backJetson != null) {
            backJetson.close();
        }
    }
}
