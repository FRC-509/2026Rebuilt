package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.charset.StandardCharsets;
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




    public class UdpVisionRelay {
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

        private volatile boolean running = false;
        private Thread worker;
        private DatagramSocket socket;

        public UdpVisionRelay(int port) {
          this.port = port;
        }

        public void start() {
            if (running) return;
            running = true;

            worker = new Thread(() -> {
              NetworkTable base = NetworkTableInstance.getDefault().getTable("Vortex").getSubTable("Vision");
            
              try {
                socket = new DatagramSocket(port);
                System.out.println("UDP relay listening on " + port);

                byte[] buf = new byte[65535];
                while (running) {
                    DatagramPacket packet = new DatagramPacket(buf, buf.length);
                    socket.receive(packet);

                    String raw = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.UTF_8);
                    Snapshot s = mapper.readValue(raw, Snapshot.class);

                    NetworkTable cam = base.getSubTable("Camera" + s.camera_index);
                    cam.getEntry("fps").setDouble(s.fps);

                    int tagCount = (s.apriltags == null) ? 0 : s.apriltags.length;
                    int objCount = (s.objects == null) ? 0 : s.objects.length;
                    cam.getEntry("apriltag_count").setDouble(tagCount);
                    cam.getEntry("object_count").setDouble(objCount);

                    boolean hasPose = s.robot_pose != null;
                    cam.getEntry("has_pose").setBoolean(hasPose);
                    if (hasPose) {
                        cam.getEntry("robot_x").setDouble(s.robot_pose.x);
                        cam.getEntry("robot_y").setDouble(s.robot_pose.y);
                        cam.getEntry("tags_used").setDouble(s.robot_pose.tags_used);
                        cam.getEntry("floor_err_avg").setDouble(s.robot_pose.floor_z_error_avg);
                }

                if (tagCount > 0) {
                    Tag t = s.apriltags[0];
                    cam.getEntry("tag0_id").setDouble(t.id);
                    cam.getEntry("tag0_x").setDouble(t.x);
                    cam.getEntry("tag0_y").setDouble(t.y);
                    cam.getEntry("tag0_z").setDouble(t.z);
                    cam.getEntry("tag0_floor_err").setDouble(t.floor_z_error);
                }

                if (objCount > 0) {
                    Obj o = s.objects[0];
                    cam.getEntry("obj0_confidence").setDouble(o.confidence);
                    cam.getEntry("obj0_x").setDouble(o.x);
                    cam.getEntry("obj0_y").setDouble(o.y);
                    cam.getEntry("obj0_z").setDouble(o.z);
                }
                }
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
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
    }


}
