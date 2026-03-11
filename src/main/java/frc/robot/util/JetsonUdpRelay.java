package frc.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

public final class JetsonUdpRelay {
	private static final ObjectMapper MAPPER = new ObjectMapper();
	private static final int MAX_PACKET_BYTES = 65535;
	private static final int MAX_CAMERAS = 2;

	private final Receiver jetson1;

	public JetsonUdpRelay(String name, int port) throws IOException {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		jetson1 = new Receiver(port, nt.getTable("/Vortex/" + name));
	}

	public void poll() {
		jetson1.poll();
	}

	public void close() {
		jetson1.close();
	}

	private static final class Receiver {
		private final DatagramChannel channel;
		private final ByteBuffer buffer = ByteBuffer.allocateDirect(MAX_PACKET_BYTES);
		private final NetworkTable jetsonTable;

		private final StringPublisher rawJsonPub;
		private final StringPublisher sourceIpPub;
		private final IntegerPublisher sourcePortPub;
		private final IntegerPublisher packetCountPub;
		private final IntegerPublisher parseErrorCountPub;

		private final CameraPublishers[] cameraPublishers = new CameraPublishers[MAX_CAMERAS];
		private final Map<Integer, Integer> sourceIndexToSlot = new HashMap<>();

		private int packetCount = 0;
		private int parseErrorCount = 0;
		private int nextSlot = 0;

		Receiver(int port, NetworkTable table) throws IOException {
			this.jetsonTable = table;
			this.channel = DatagramChannel.open();
			this.channel.configureBlocking(false);
			this.channel.bind(new InetSocketAddress(port));

			rawJsonPub = table.getStringTopic("raw_json").publish();
			sourceIpPub = table.getStringTopic("source_ip").publish();
			sourcePortPub = table.getIntegerTopic("source_port").publish();
			packetCountPub = table.getIntegerTopic("packet_count").publish();
			parseErrorCountPub = table.getIntegerTopic("parse_error_count").publish();

			for (int slot = 0; slot < MAX_CAMERAS; slot++) {
				cameraPublishers[slot] = new CameraPublishers(table.getSubTable("camera" + slot));
			}
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
						int sourceCameraIndex = root.path("camera_index").asInt(-1);
						if (sourceCameraIndex < 0) {
							parseErrorCount++;
							parseErrorCountPub.set(parseErrorCount);
							continue;
						}

						Integer slot = sourceIndexToSlot.get(sourceCameraIndex);
						if (slot == null) {
							if (nextSlot >= MAX_CAMERAS) {
								continue;
							}
							slot = nextSlot++;
							sourceIndexToSlot.put(sourceCameraIndex, slot);
						}

						CameraPublishers pubs = cameraPublishers[slot];

						pubs.schemaVersionPub.set(root.path("schema_version").asInt(0));
						pubs.cameraIndexPub.set(sourceCameraIndex);
						pubs.fpsPub.set(root.path("fps").asDouble(0.0));

						JsonNode apriltags = root.path("apriltags");
						JsonNode objects = root.path("objects");
						pubs.apriltagsJsonPub.set(apriltags.isMissingNode() ? "[]" : apriltags.toString());
						pubs.objectsJsonPub.set(objects.isMissingNode() ? "[]" : objects.toString());

						JsonNode robotPose = root.path("robot_pose");
						boolean hasPose = !robotPose.isMissingNode() && !robotPose.isNull();
						pubs.hasPosePub.set(hasPose);

						if (hasPose) {
							pubs.robotXPub.set(robotPose.path("x").asDouble(0.0));
							pubs.robotYPub.set(robotPose.path("y").asDouble(0.0));
							pubs.tagsUsedPub.set(robotPose.path("tags_used").asInt(0));
							pubs.floorErrPub.set(robotPose.path("floor_z_error_avg").asDouble(0.0));
						} else {
							pubs.robotXPub.set(0.0);
							pubs.robotYPub.set(0.0);
							pubs.tagsUsedPub.set(0);
							pubs.floorErrPub.set(0.0);
						}

						pubs.packetCount++;
						pubs.packetCountPub.set(pubs.packetCount);
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

	private static final class CameraPublishers {
		final IntegerPublisher schemaVersionPub;
		final DoublePublisher fpsPub;
		final IntegerPublisher cameraIndexPub;
		final BooleanPublisher hasPosePub;
		final DoublePublisher robotXPub;
		final DoublePublisher robotYPub;
		final IntegerPublisher tagsUsedPub;
		final DoublePublisher floorErrPub;
		final StringPublisher apriltagsJsonPub;
		final StringPublisher objectsJsonPub;
		final IntegerPublisher packetCountPub;

		int packetCount = 0;

		CameraPublishers(NetworkTable table) {
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
		}
	}
}
