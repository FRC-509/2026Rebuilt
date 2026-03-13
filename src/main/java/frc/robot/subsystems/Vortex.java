package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants.Drive;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.JetsonUdpRelay;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Vortex {
    private static final double JETSON_STALE_SECONDS = 0.5;

    private static final String FRONT_JETSON_TABLE_NAME = "VortexFront";
    private static final String BACK_JETSON_TABLE_NAME = "VortexBack";
    
    private final NetworkTable frontJetsonTable;
    private final NetworkTable backJetsonTable;

    private JetsonUdpRelay frontJetson;
    private JetsonUdpRelay backJetson;

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier intakeExtension;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d;

    private double lastJetsonMeasurementTimestamp = Double.NaN;
    private double lastLimelightMeasurementTimestamp = Double.NaN;

    public Vortex(SwerveDrive swerve, Pose2d initialPose, DoubleSupplier intakeExtension) {
        this.swerveDrive = swerve;
        this.intakeExtension = intakeExtension;

        try {
            this.frontJetson = new JetsonUdpRelay(FRONT_JETSON_TABLE_NAME, 5091);
            this.backJetson = new JetsonUdpRelay(BACK_JETSON_TABLE_NAME, 5092);
        } catch (Exception e) {
            DriverStation.reportError("Jetson failed to initialize.",false);
            this.frontJetson = null;
            this.backJetson = null;
        }

        this.frontJetsonTable = NetworkTableInstance.getDefault().getTable("/Vortex/" + FRONT_JETSON_TABLE_NAME);
        this.backJetsonTable = NetworkTableInstance.getDefault().getTable("/Vortex/" + BACK_JETSON_TABLE_NAME);

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
        return SwerveDrive.getAlliance().equals(Alliance.Red) //TODO: confirm blue alliance has 0,0
            ? new Translation2d(Constants.Field.kFullFieldLength,Constants.Field.kFieldWidth).minus(getEstimatedGlobalPosition())
            : getEstimatedGlobalPosition();
    }

    public Translation2d updatePositionEstimate() {
        poseEstimator.update(swerveDrive.getYaw(), swerveDrive.getModulePositions());
        applyJetsonMeasurement("Vortex/VortexFront");
        applyJetsonMeasurement("Vortex/VortexBack");


        
        // LimelightHelpers.SetRobotOrientation(Constants.Vortex.kLimelightName, swerveDrive.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        // PoseEstimate mt2 = SwerveDrive.getAlliance().equals(Alliance.Red)
        //     ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(Constants.Vortex.kLimelightName) // TODO: double check which to use, conditional needed?
        //     : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vortex.kLimelightName);
   

        // // if our angular velocity is greater than 360 degrees per second or theres no tags, ignore vision updates
        // if(!(mt2 == null
        //         || mt2.pose == null
        //         || Math.abs(swerveDrive.getAngularVelocity()) > 360 // TODO: tune w threshold
        //         || mt2.tagCount == 0 
        //         || mt2.timestampSeconds <= lastLimelightMeasurementTimestamp
        //         || LimelightHelpers.getCurrentPipelineIndex(Constants.Vortex.kLimelightName) != 0)) {

            // mt2.pose = new Pose2d( // TODO: double check sign of vector
            //     mt2.pose.getX() + intakeExtension.getAsDouble() * Math.cos(swerveDrive.getYaw().getRadians()), // account for extending intake by 
            //     mt2.pose.getY() + intakeExtension.getAsDouble() * -Math.sin(swerveDrive.getYaw().getRadians()), // adding rotated extension vector
            //     swerveDrive.getYaw());

        //     poseEstimator.addVisionMeasurement(
        //         mt2.pose,
        //         mt2.timestampSeconds,
        //         Constants.Vortex.kLimelightMeasurementStdDevs);
        //     lastLimelightMeasurementTimestamp = mt2.timestampSeconds;
        // }

        return swerveDrive.getEstimatedPose().getTranslation();
    }

    private void applyJetsonMeasurement(String jetsonCameraString) {
        if (!hasFreshJetsonPose()) {
            return;
        }

        double measurementTimestamp = frontJetsonTable.getEntry("last_packet_timestamp").getDouble(Double.NaN);
        if (Double.isNaN(measurementTimestamp) || measurementTimestamp <= lastJetsonMeasurementTimestamp) {
            return;
        }

        double y = Math.abs(frontJetsonTable.getEntry("/robot/x").getDouble(0.0));
        double x = Math.abs(frontJetsonTable.getEntry("/robot/y").getDouble(0.0));

        poseEstimator.addVisionMeasurement(
            new Pose2d(x,y,swerveDrive.getYaw()),
            measurementTimestamp,
            Constants.Vortex.kVortexMeasurementStdDevs);
        lastJetsonMeasurementTimestamp = measurementTimestamp;
    }

    private boolean hasFreshJetsonPose() {
        double lastPacketTimestamp = frontJetsonTable.getEntry("last_packet_timestamp").getDouble(Double.NaN);
        return !Double.isNaN(lastPacketTimestamp)
            && (Timer.getFPGATimestamp() - lastPacketTimestamp) <= JETSON_STALE_SECONDS
            && frontJetsonTable.getEntry("robot/has_pose").getBoolean(false)
            && backJetsonTable.getEntry("robot/has_pose").getBoolean(false);
    }

    public Translation2d getLatestJetsonPose() {

        double frontx1 = Math.abs(frontJetsonTable.getEntry("robot/x").getDouble(0.0));
        double fronty1 = Math.abs(frontJetsonTable.getEntry("robot/y").getDouble(0.0));
        
        double backx1 = Math.abs(frontJetsonTable.getEntry("robot/x").getDouble(0.0));
        double backy1 = Math.abs(frontJetsonTable.getEntry("robot/y").getDouble(0.0));

        double avgx = (frontx1+backx1)/2;
        double avgy = (fronty1+backy1)/2;
        return new Translation2d(
            avgy,
            avgx);
    }

    public void pollJetsons(){
        frontJetson.poll();
        backJetson.poll();
        Translation2d jetsonPosition = getLatestJetsonPose();
        SmartDashboard.putNumber("JetsonX",jetsonPosition.getX());
        SmartDashboard.putNumber("Jetsony",jetsonPosition.getY());
        
        Pose2d awd = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("EstimatorX",awd.getX());
        SmartDashboard.putNumber("EstimatorY",awd.getY());
    }

    public void closeJetsons(){
        frontJetson.close();
        backJetson.close();
    }
}
