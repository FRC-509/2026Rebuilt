package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.JetsonUdpRelay;
import frc.robot.util.JetsonUdpRelay.RelayState;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.math.GeometryUtils;

public class Vortex {
    private static final String FRONT_JETSON_TABLE_NAME = "VortexFront";
    private static final String BACK_JETSON_TABLE_NAME = "VortexBack";
    private static final String VORTEX_TABLE_NAME = "Vortex";
    private static final double RED_ALLIANCE_POSITION_ALPHA = 0.1;

    private final NetworkTable vortexTable;

    private final JetsonUdpRelay frontJetson;
    private final JetsonUdpRelay backJetson;

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier intakeExtension;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private double lastFrontJetsonMeasurementTimestamp = Double.NaN;
    private double lastBackJetsonMeasurementTimestamp = Double.NaN;
    private double lastLimelightMeasurementTimestamp = Double.NaN;
    private Translation2d estimatedGlobalPosition = Translation2d.kZero;
    private Translation2d filteredRedAlliancePosition = null;

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
        this.vortexTable = NetworkTableInstance.getDefault().getTable(VORTEX_TABLE_NAME);

        this.poseEstimator = new SwerveDrivePoseEstimator(
            swerve.kinematics,
            swerve.getYaw(),
            swerve.getModulePositions(),
            initialPose);
        this.field2d = new Field2d();
        this.aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        swerve.resetOdometry(initialPose);
        swerve.setEstimatedPoseSupplier(poseEstimator::getEstimatedPosition);
        field2d.setRobotPose(initialPose);
        estimatedGlobalPosition = computeEstimatedGlobalPosition();

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
        return estimatedGlobalPosition;
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

    public Field2d getField() {
        return field2d;
    }

    public double getIntakeExtensionMeters() {
        return intakeExtension.getAsDouble();
    }

    public boolean hasFrontJetsonPose() {
        return frontJetson != null && frontJetson.hasPose();
    }

    public boolean hasBackJetsonPose() {
        return backJetson != null && backJetson.hasPose();
    }

    public boolean hasLimelightPose() {
        return !Double.isNaN(lastLimelightMeasurementTimestamp);
    }

    public double getLastFrontJetsonMeasurementTimestamp() {
        return frontJetson == null ? Double.NaN : frontJetson.getLastPacketTimestamp();
    }

    public double getLastBackJetsonMeasurementTimestamp() {
        return backJetson == null ? Double.NaN : backJetson.getLastPacketTimestamp();
    }

    public double getLastLimelightMeasurementTimestamp() {
        return lastLimelightMeasurementTimestamp;
    }

    public RelayState getFrontJetsonState() {
        return frontJetson == null ? JetsonUdpRelay.RelayState.empty() : frontJetson.getState();
    }

    public RelayState getBackJetsonState() {
        return backJetson == null ? JetsonUdpRelay.RelayState.empty() : backJetson.getState();
    }

    public Translation2d updatePositionEstimate() {
        poseEstimator.update(swerveDrive.getYaw(), swerveDrive.getModulePositions());

        lastFrontJetsonMeasurementTimestamp = applyJetsonMeasurement(frontJetson, lastFrontJetsonMeasurementTimestamp);
        lastBackJetsonMeasurementTimestamp = applyJetsonMeasurement(backJetson, lastBackJetsonMeasurementTimestamp);
        applyFrontLimelightMeasurement();
        estimatedGlobalPosition = computeEstimatedGlobalPosition();
        postVortexToNT();

        return getEstimatedGlobalPosition();
    }

    private double applyJetsonMeasurement(JetsonUdpRelay jetsonRelay, double lastMeasurementTimestamp) {
        if (jetsonRelay == null || !jetsonRelay.hasPose()) {
            return lastMeasurementTimestamp;
        }

        double measurementTimestamp = jetsonRelay.getLastPacketTimestamp();
        if (Double.isNaN(measurementTimestamp) || measurementTimestamp <= lastMeasurementTimestamp) {
            return lastMeasurementTimestamp;
        }
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

        boolean isRedAlliance = SwerveDrive.getAlliance() == Alliance.Red;
        PoseEstimate poseEstimate = isRedAlliance
            ? LimelightHelpers.getBotPoseEstimate_wpiRed(Constants.Vortex.kFrontLimelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vortex.kFrontLimelightName);

        if (poseEstimate == null
                || poseEstimate.pose == null
                || Math.abs(swerveDrive.getAngularVelocity()) > 360
                || poseEstimate.tagCount == 0
                || poseEstimate.timestampSeconds <= lastLimelightMeasurementTimestamp
                || LimelightHelpers.getCurrentPipelineIndex(Constants.Vortex.kFrontLimelightName) != 0) {
            return;
        }

        Pose2d limelightPose = isRedAlliance
            ? GeometryUtils.flipFieldPose(poseEstimate.pose)
            : poseEstimate.pose;

        poseEstimator.addVisionMeasurement(
            new Pose2d(limelightPose.getTranslation(), swerveDrive.getYaw()),
            poseEstimate.timestampSeconds,
            Constants.Vortex.kLimelightMeasurementStdDevs);
        lastLimelightMeasurementTimestamp = poseEstimate.timestampSeconds;
    }

    private Pose2d getJetsonPose(JetsonUdpRelay jetsonRelay) {
        return new Pose2d(
            jetsonRelay.getRobotX(),
            jetsonRelay.getRobotY(),
            swerveDrive.getYaw());
    }

    private Matrix<N3, N1> getJetsonMeasurementStdDevs(JetsonUdpRelay jetsonRelay) {
        Matrix<N3, N1> baseStdDevs = Constants.Vortex.kJetsonBaseMeasurementStdDevs;
        double floorErrorMeters = Math.abs(jetsonRelay.getFloorZErrorAvg());
        double excessFloorError = Math.max(0.0, floorErrorMeters - Constants.Vortex.kJetsonFloorErrorTrustThresholdMeters);
        double multiplier = 1.0 + (excessFloorError * Constants.Vortex.kJetsonFloorErrorStdDevScale);
        multiplier = Math.min(multiplier, Constants.Vortex.kJetsonMaxStdDevMultiplier);

        return VecBuilder.fill(
            baseStdDevs.get(0, 0) * multiplier,
            baseStdDevs.get(1, 0) * multiplier,
            baseStdDevs.get(2, 0) * multiplier);
    }

    public Translation2d getLatestJetsonPose() {
        Translation2d sum = Translation2d.kZero;
        int measurementCount = 0;

        if (hasFrontJetsonPose()) {
            sum = sum.plus(getJetsonPose(frontJetson).getTranslation());
            measurementCount++;
        }
        if (hasBackJetsonPose()) {
            sum = sum.plus(getJetsonPose(backJetson).getTranslation());
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

        updatePositionEstimate();
    }

    private void postVortexToNT() {
        Pose2d estimatedGlobalPose = new Pose2d(getEstimatedGlobalPosition(), getEstimatedPose().getRotation());
        Pose2d estimatedAlliancePose = getEstimatedAlliancePose();

        vortexTable.getEntry("EstimatedPose").setDoubleArray(new double[] {
            estimatedAlliancePose.getX(),
            estimatedAlliancePose.getY(),
            estimatedAlliancePose.getRotation().getDegrees()
        });
        vortexTable.getEntry("EstimatedGlobalPose").setDoubleArray(new double[] {
            estimatedGlobalPose.getX(),
            estimatedGlobalPose.getY(),
            estimatedGlobalPose.getRotation().getDegrees()
        });
        vortexTable.getEntry("HasFrontJetsonPose").setBoolean(hasFrontJetsonPose());
        vortexTable.getEntry("HasBackJetsonPose").setBoolean(hasBackJetsonPose());
        vortexTable.getEntry("HasLimelightPose").setBoolean(hasLimelightPose());

        field2d.setRobotPose(estimatedGlobalPose);
        if (frontJetson != null) {
            field2d.getObject("front_jetson_pose").setPose(getJetsonPose(frontJetson));
        }
        if (backJetson != null) {
            field2d.getObject("back_jetson_pose").setPose(getJetsonPose(backJetson));
        }

        Pose2d mt2TagSpacePose = getFrontLimelightTagSpacePose();
        if (mt2TagSpacePose != null) {
            field2d.getObject("front_limelight_tagspace_pose").setPose(mt2TagSpacePose);
            vortexTable.getEntry("FrontLimelightTagSpacePose").setDoubleArray(new double[] {
                mt2TagSpacePose.getX(),
                mt2TagSpacePose.getY(),
                mt2TagSpacePose.getRotation().getDegrees()
            });
        }
    }

    private Translation2d toAllianceRelative(Translation2d globalPosition) {
        return SwerveDrive.getAlliance() == Alliance.Red
            ? new Translation2d(
                Constants.Field.kFullFieldLength - globalPosition.getX(),
                Constants.Field.kFieldWidth - globalPosition.getY())
            : globalPosition;
    }

    private Translation2d computeEstimatedGlobalPosition() {
        Translation2d rawEstimatedPosition = poseEstimator.getEstimatedPosition().getTranslation();
        Translation2d mirroredPosition = new Translation2d(
            rawEstimatedPosition.getX(),
            Constants.Field.kFieldWidth - rawEstimatedPosition.getY());

        if (SwerveDrive.getAlliance() != Alliance.Red) {
            filteredRedAlliancePosition = null;
            return mirroredPosition;
        }

        if (filteredRedAlliancePosition == null) {
            filteredRedAlliancePosition = mirroredPosition;
            return filteredRedAlliancePosition;
        }

        filteredRedAlliancePosition = new Translation2d(
            (RED_ALLIANCE_POSITION_ALPHA * mirroredPosition.getX())
                + ((1.0 - RED_ALLIANCE_POSITION_ALPHA) * filteredRedAlliancePosition.getX()),
            (RED_ALLIANCE_POSITION_ALPHA * mirroredPosition.getY())
                + ((1.0 - RED_ALLIANCE_POSITION_ALPHA) * filteredRedAlliancePosition.getY()));
        return filteredRedAlliancePosition;
    }

    private Pose2d toAllianceRelative(Pose2d globalPose) {
        if (SwerveDrive.getAlliance() != Alliance.Red) {
            return globalPose;
        }

        return GeometryUtils.flipFieldPose(globalPose);
    }

    private Pose2d getFrontLimelightTagSpacePose() {
        double fiducialId = LimelightHelpers.getFiducialID(Constants.Vortex.kFrontLimelightName);
        if (fiducialId < 0) {
            return null;
        }

        Pose3d botPoseTargetSpace = LimelightHelpers.getBotPose3d_TargetSpace(Constants.Vortex.kFrontLimelightName);
        if (botPoseTargetSpace == null) {
            return null;
        }

        return aprilTagFieldLayout.getTagPose((int) fiducialId)
            .map(tagPose -> new Pose2d(
                tagPose.getX() - botPoseTargetSpace.getX(),
                tagPose.getY() - botPoseTargetSpace.getY(),
                Rotation2d.fromRadians(-botPoseTargetSpace.getRotation().getZ())))
            .orElse(null);
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
