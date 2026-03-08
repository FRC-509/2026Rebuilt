package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
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
    private final UdpVisionRelay udpVisionRelay = new UdpVisionRelay();

    public Vortex(SwerveDrive swerve, Pose2d initialPose, DoubleSupplier intakeExtension) {
        this.poseEstimator = new SwerveDrivePoseEstimator(
            swerve.kinematics, 
            swerve.getYaw(),
            swerve.getModulePositions(), 
            initialPose);

        this.swerveDrive = swerve;
        this.intakeExtension = intakeExtension;

        poseEstimator.addVisionMeasurement(initialPose, Timer.getFPGATimestamp());

        
        LimelightHelpers.setCameraPose_RobotSpace( //TODO: get real values
            Constants.Vortex.kLimelightName,
            0,
            0, 
            0,
            0,
            -10, 
            -180);
        LimelightHelpers.setPipelineIndex(Constants.Vortex.kLimelightName, 0);
        udpVisionRelay.start();
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
        if(!(mt2 == null
                || mt2.pose == null
                || Math.abs(swerveDrive.getAngularVelocity()) > 360 // TODO: tune w threshold
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

    public void postVortexToNT() {
        udpVisionRelay.start();
    }




    public class UdpVisionRelay {
        public static final int DEFAULT_PORT = 5091;

        public static class Tag {
            public int id;
            public double x;
            public double y;
            public double z;
            public double floor_z_error;
        }

        public static class Obj {
            public String class_name;
            public double confidence;
            public double[] bbox;
            public double x;
            public double y;
            public double z;
        }

        public static class RobotPose {
            public double x;
            public double y;
            public int tags_used;
            public double floor_z_error_avg;
        }

        public static class Snapshot {
            public int camera_index;
            public double fps;
            public Tag[] apriltags;
            public Obj[] objects;
            public RobotPose robot_pose;
        }

        private final int port;
        private final ObjectMapper mapper = new ObjectMapper();
        private final Map<Integer, Integer> lastTagCountByCamera = new HashMap<>();
        private final Map<Integer, Integer> lastObjectCountByCamera = new HashMap<>();

        private volatile boolean running = false;
        private Thread worker;
        private DatagramSocket socket;

        public UdpVisionRelay() {
            this(DEFAULT_PORT);
        }

        public UdpVisionRelay(int port) {
          this.port = DEFAULT_PORT;
        }

        public void start() {
            if (running) return;
            running = true;

            worker = new Thread(() -> {
              NetworkTable base = NetworkTableInstance.getDefault().getTable("Vortex").getSubTable("Vision");
              double parseErrorCount = 0;
              base.getEntry("relay_running").setBoolean(true);
              base.getEntry("relay_port").setDouble(port);
            
              try {
                socket = new DatagramSocket(port);
                System.out.println("UDP relay listening on " + port);

                byte[] buf = new byte[65535];
                while (running) {
                    DatagramPacket packet = new DatagramPacket(buf, buf.length);
                    socket.receive(packet);

                    String raw = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.UTF_8);
                    base.getEntry("last_packet_seconds").setDouble(Timer.getFPGATimestamp());
                    base.getEntry("last_packet_bytes").setDouble(packet.getLength());

                    Snapshot s;
                    try {
                        s = mapper.readValue(raw, Snapshot.class);
                    } catch (Exception parseEx) {
                        parseErrorCount += 1;
                        base.getEntry("parse_error_count").setDouble(parseErrorCount);
                        base.getEntry("last_parse_error").setString(parseEx.getMessage());
                        continue;
                    }

                    NetworkTable cam = base.getSubTable("Camera" + s.camera_index);
                    NetworkTable directCam = NetworkTableInstance.getDefault().getTable(cameraTableName(s.camera_index));
                    cam.getEntry("raw_json").setString(raw);
                    cam.getEntry("camera_index").setDouble(s.camera_index);
                    cam.getEntry("fps").setDouble(s.fps);
                    directCam.getEntry("raw_json").setString(raw);
                    directCam.getEntry("camera_index").setDouble(s.camera_index);
                    directCam.getEntry("fps").setDouble(s.fps);

                    int tagCount = (s.apriltags == null) ? 0 : s.apriltags.length;
                    int objCount = (s.objects == null) ? 0 : s.objects.length;
                    cam.getEntry("apriltag_count").setDouble(tagCount);
                    cam.getEntry("object_count").setDouble(objCount);
                    directCam.getEntry("apriltag_count").setDouble(tagCount);
                    directCam.getEntry("object_count").setDouble(objCount);

                    NetworkTable pose = cam.getSubTable("robot_pose");
                    NetworkTable directPose = directCam.getSubTable("robot_pose");
                    boolean hasPose = s.robot_pose != null;
                    cam.getEntry("has_pose").setBoolean(hasPose);
                    pose.getEntry("has_pose").setBoolean(hasPose);
                    directCam.getEntry("has_pose").setBoolean(hasPose);
                    directPose.getEntry("has_pose").setBoolean(hasPose);
                    if (hasPose) {
                        pose.getEntry("x").setDouble(s.robot_pose.x);
                        pose.getEntry("y").setDouble(s.robot_pose.y);
                        pose.getEntry("tags_used").setDouble(s.robot_pose.tags_used);
                        pose.getEntry("floor_z_error_avg").setDouble(s.robot_pose.floor_z_error_avg);
                        directPose.getEntry("x").setDouble(s.robot_pose.x);
                        directPose.getEntry("y").setDouble(s.robot_pose.y);
                        directPose.getEntry("tags_used").setDouble(s.robot_pose.tags_used);
                        directPose.getEntry("floor_z_error_avg").setDouble(s.robot_pose.floor_z_error_avg);
                        directCam.getEntry("Position").setDoubleArray(new double[] {s.robot_pose.x, s.robot_pose.y});
                    }

                    NetworkTable apriltags = cam.getSubTable("apriltags");
                    NetworkTable directApriltags = directCam.getSubTable("apriltags");
                    apriltags.getEntry("count").setDouble(tagCount);
                    directApriltags.getEntry("count").setDouble(tagCount);
                    for (int i = 0; i < tagCount; i++) {
                        Tag t = s.apriltags[i];
                        NetworkTable tag = apriltags.getSubTable("tag" + i);
                        NetworkTable directTag = directApriltags.getSubTable("tag" + i);
                        tag.getEntry("present").setBoolean(true);
                        tag.getEntry("id").setDouble(t.id);
                        tag.getEntry("x").setDouble(t.x);
                        tag.getEntry("y").setDouble(t.y);
                        tag.getEntry("z").setDouble(t.z);
                        tag.getEntry("floor_z_error").setDouble(t.floor_z_error);
                        directTag.getEntry("present").setBoolean(true);
                        directTag.getEntry("id").setDouble(t.id);
                        directTag.getEntry("x").setDouble(t.x);
                        directTag.getEntry("y").setDouble(t.y);
                        directTag.getEntry("z").setDouble(t.z);
                        directTag.getEntry("floor_z_error").setDouble(t.floor_z_error);
                    }
                    int previousTagCount = lastTagCountByCamera.getOrDefault(s.camera_index, 0);
                    for (int i = tagCount; i < previousTagCount; i++) {
                        apriltags.getSubTable("tag" + i).getEntry("present").setBoolean(false);
                        directApriltags.getSubTable("tag" + i).getEntry("present").setBoolean(false);
                    }
                    lastTagCountByCamera.put(s.camera_index, tagCount);

                    NetworkTable objects = cam.getSubTable("objects");
                    NetworkTable directObjects = directCam.getSubTable("objects");
                    objects.getEntry("count").setDouble(objCount);
                    directObjects.getEntry("count").setDouble(objCount);
                    for (int i = 0; i < objCount; i++) {
                        Obj o = s.objects[i];
                        NetworkTable obj = objects.getSubTable("obj" + i);
                        NetworkTable directObj = directObjects.getSubTable("obj" + i);
                        obj.getEntry("present").setBoolean(true);
                        obj.getEntry("class_name").setString(o.class_name == null ? "" : o.class_name);
                        obj.getEntry("confidence").setDouble(o.confidence);
                        obj.getEntry("bbox").setDoubleArray(o.bbox == null ? new double[0] : o.bbox);
                        obj.getEntry("x").setDouble(o.x);
                        obj.getEntry("y").setDouble(o.y);
                        obj.getEntry("z").setDouble(o.z);
                        directObj.getEntry("present").setBoolean(true);
                        directObj.getEntry("class_name").setString(o.class_name == null ? "" : o.class_name);
                        directObj.getEntry("confidence").setDouble(o.confidence);
                        directObj.getEntry("bbox").setDoubleArray(o.bbox == null ? new double[0] : o.bbox);
                        directObj.getEntry("x").setDouble(o.x);
                        directObj.getEntry("y").setDouble(o.y);
                        directObj.getEntry("z").setDouble(o.z);
                    }
                    int previousObjectCount = lastObjectCountByCamera.getOrDefault(s.camera_index, 0);
                    for (int i = objCount; i < previousObjectCount; i++) {
                        objects.getSubTable("obj" + i).getEntry("present").setBoolean(false);
                        directObjects.getSubTable("obj" + i).getEntry("present").setBoolean(false);
                    }
                    lastObjectCountByCamera.put(s.camera_index, objCount);
                }
            } catch (Exception e) {
                base.getEntry("relay_running").setBoolean(false);
                base.getEntry("last_relay_error").setString(e.getMessage() == null ? e.toString() : e.getMessage());
                e.printStackTrace();
            } finally {
                base.getEntry("relay_running").setBoolean(false);
                if (socket != null && !socket.isClosed()) {
                socket.close();
                }
            }
            }, "UdpVisionRelay");

            worker.setDaemon(true);
            worker.start();
        }

        public void stop() {
            running = false;
            if (socket != null && !socket.isClosed()) {
                socket.close();
            }
        }

        private String cameraTableName(int cameraIndex) {
            if (cameraIndex == 0) return "VortexFront";
            if (cameraIndex == 1) return "VortexBack";
            return "VortexCamera" + cameraIndex;
        }
    }


}
