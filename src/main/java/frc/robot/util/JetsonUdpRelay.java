package frc.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanPublisher;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.charset.StandardCharsets;

public final class JetsonUdpRelay {
  private static final ObjectMapper MAPPER = new ObjectMapper();
  private static final int MAX_PACKET_BYTES = 65535;

  private final Receiver jetson1;
  private final Receiver jetson2;

  public JetsonUdpRelay() throws IOException {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    jetson1 = new Receiver(5091, nt.getTable("/Vortex/Jetson1"));
    jetson2 = new Receiver(5092, nt.getTable("/Vortex/Jetson2"));
  }

  public void poll() {
    jetson1.poll();
    jetson2.poll();
  }

  public void close() {
    jetson1.close();
    jetson2.close();
  }

  private static final class Receiver {
    private final int port;
    private final DatagramChannel channel;
    private final ByteBuffer buffer = ByteBuffer.allocateDirect(MAX_PACKET_BYTES);

    private final StringPublisher rawJsonPub;
    private final StringPublisher sourceIpPub;
    private final IntegerPublisher sourcePortPub;
    private final IntegerPublisher schemaVersionPub;
    private final DoublePublisher fpsPub;
    private final IntegerPublisher cameraIndexPub;
    private final BooleanPublisher hasPosePub;
    private final DoublePublisher robotXPub;
    private final DoublePublisher robotYPub;
    private final IntegerPublisher tagsUsedPub;
    private final DoublePublisher floorErrPub;
    private final StringPublisher apriltagsJsonPub;
    private final StringPublisher objectsJsonPub;
    private final IntegerPublisher packetCountPub;
    private final IntegerPublisher parseErrorCountPub;

    private int packetCount = 0;
    private int parseErrorCount = 0;

    Receiver(int port, NetworkTable table) throws IOException {
      this.port = port;
      this.channel = DatagramChannel.open();
      this.channel.configureBlocking(false);
      this.channel.bind(new InetSocketAddress(port));

      rawJsonPub = table.getStringTopic("raw_json").publish();
      sourceIpPub = table.getStringTopic("source_ip").publish();
      sourcePortPub = table.getIntegerTopic("source_port").publish();
      schemaVersionPub = table.getIntegerTopic("schema_version").publish();
      fpsPub = table.getDoubleTopic("fps").publish();
      cameraIndexPub = table.getIntegerTopic("camera_index").publish();
      hasPosePub = table.getBooleanTopic("robot/has_pose").publish();
      robotXPub = table.getDoubleTopic("robot/x").publish();
      robotYPub = table.getDoubleTopic("robot/y").publish();
      tagsUsedPub = table.getIntegerTopic("robot/tags_used").publish();
      floorErrPub = table.getDoubleTopic("robot/floor_z_error_avg").publish();
      apriltagsJsonPub = table.getStringTopic("apriltags_json").publish();
      objectsJsonPub = table.getStringTopic("objects_json").publish();
      packetCountPub = table.getIntegerTopic("packet_count").publish();
      parseErrorCountPub = table.getIntegerTopic("parse_error_count").publish();
    }

    void poll() {
      try {
        while (true) {
          buffer.clear();
          SocketAddress remote = channel.receive(buffer);
          if (remote == null) {
            break;
          }

          buffer.flip();
          String json = StandardCharsets.UTF_8.decode(buffer).toString();
          packetCount++;
          packetCountPub.set(packetCount);
          rawJsonPub.set(json);

          if (remote instanceof InetSocketAddress addr) {
            sourceIpPub.set(addr.getAddress().getHostAddress());
            sourcePortPub.set(addr.getPort());
          }

          try {
            JsonNode root = MAPPER.readTree(json);

            schemaVersionPub.set(root.path("schema_version").asInt(0));
            cameraIndexPub.set(root.path("camera_index").asInt(-1));
            fpsPub.set(root.path("fps").asDouble(0.0));

            JsonNode apriltags = root.path("apriltags");
            JsonNode objects = root.path("objects");
            apriltagsJsonPub.set(apriltags.isMissingNode() ? "[]" : apriltags.toString());
            objectsJsonPub.set(objects.isMissingNode() ? "[]" : objects.toString());

            JsonNode robotPose = root.path("robot_pose");
            boolean hasPose = !robotPose.isMissingNode() && !robotPose.isNull();
            hasPosePub.set(hasPose);

            if (hasPose) {
              robotXPub.set(robotPose.path("x").asDouble(0.0));
              robotYPub.set(robotPose.path("y").asDouble(0.0));
              tagsUsedPub.set(robotPose.path("tags_used").asInt(0));
              floorErrPub.set(robotPose.path("floor_z_error_avg").asDouble(0.0));
            } else {
              robotXPub.set(0.0);
              robotYPub.set(0.0);
              tagsUsedPub.set(0);
              floorErrPub.set(0.0);
            }
          } catch (Exception parseErr) {
            parseErrorCount++;
            parseErrorCountPub.set(parseErrorCount);
          }
        }
      } catch (IOException ioErr) {
        parseErrorCount++;
        parseErrorCountPub.set(parseErrorCount);
      }
    }

    void close() {
      try {
        channel.close();
      } catch (IOException ignored) {
      }
    }
  }
}
