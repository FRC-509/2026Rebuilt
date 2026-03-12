package frc.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.AsynchronousCloseException;
import java.nio.channels.DatagramChannel;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

public final class JetsonUdpRelay {
	private static final ObjectMapper MAPPER = new ObjectMapper();
	private static final int MAX_PACKET_BYTES = 65535;
	private static final int MAX_CAMERAS = 2;
	private static final double STALE_PACKET_SECONDS = 0.5;

	private final Receiver jetson1;

	public JetsonUdpRelay(String name, int port) throws IOException {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		jetson1 = new Receiver(port, nt.getTable("/Vortex/" + name));
	}

	public void poll() {
		// Receive/parsing runs on the worker thread.
	}

	public void close() {
		jetson1.close();
	}

	private static final class Receiver {
		private final DatagramChannel channel;
		private final NetworkTable jetsonTable;
		private final Thread worker;

		private final StringPublisher rawJsonPub;
		private final StringPublisher sourceIpPub;
		private final IntegerPublisher sourcePortPub;
		private final IntegerPublisher packetCountPub;
		private final IntegerPublisher parseErrorCountPub;
		private final IntegerPublisher ioErrorCountPub;
		private final DoublePublisher lastPacketTimestampPub;
		private final DoublePublisher packetAgePub;
		private final BooleanPublisher stalePub;
		private final BooleanPublisher relayRunningPub;
		private final BooleanPublisher hasPosePub;
		private final DoublePublisher robotXPub;
		private final DoublePublisher robotYPub;
		private final IntegerPublisher tagsUsedPub;
		private final DoublePublisher floorErrPub;

		private final CameraPublishers[] cameraPublishers = new CameraPublishers[MAX_CAMERAS];
		private final Map<Integer, Integer> sourceIndexToSlot = new HashMap<>();

		private volatile boolean running = true;
		private volatile int packetCount = 0;
		private volatile int parseErrorCount = 0;
		private volatile int ioErrorCount = 0;
		private int nextSlot = 0;
		private volatile double lastPacketTimestamp = Double.NaN;

		Receiver(int port, NetworkTable table) throws IOException {
			this.jetsonTable = table;
			this.channel = DatagramChannel.open();
			this.channel.configureBlocking(true);
			this.channel.bind(new InetSocketAddress(port));

			rawJsonPub = table.getStringTopic("raw_json").publish();
			sourceIpPub = table.getStringTopic("source_ip").publish();
			sourcePortPub = table.getIntegerTopic("source_port").publish();
			packetCountPub = table.getIntegerTopic("packet_count").publish();
			parseErrorCountPub = table.getIntegerTopic("parse_error_count").publish();
			ioErrorCountPub = table.getIntegerTopic("io_error_count").publish();
			lastPacketTimestampPub = table.getDoubleTopic("last_packet_timestamp").publish();
			packetAgePub = table.getDoubleTopic("packet_age_seconds").publish();
			stalePub = table.getBooleanTopic("stale").publish();
			relayRunningPub = table.getBooleanTopic("relay_running").publish();
			hasPosePub = table.getBooleanTopic("robot/has_pose").publish();
			robotXPub = table.getDoubleTopic("robot/x").publish();
			robotYPub = table.getDoubleTopic("robot/y").publish();
			tagsUsedPub = table.getIntegerTopic("robot/tags_used").publish();
			floorErrPub = table.getDoubleTopic("robot/floor_z_error_avg").publish();

			for (int slot = 0; slot < MAX_CAMERAS; slot++) {
				cameraPublishers[slot] = new CameraPublishers(table.getSubTable("camera" + slot));
			}

			relayRunningPub.set(true);
			stalePub.set(true);

			worker = new Thread(this::run, "JetsonUdpRelay-" + port);
			worker.setDaemon(true);
			worker.start();
		}

		void close() {
			running = false;
			relayRunningPub.set(false);

			try {
				channel.close();
			} catch (IOException ignored) {
			}

			try {
				worker.join(100);
			} catch (InterruptedException interrupted) {
				Thread.currentThread().interrupt();
			}
		}

		private void run() {
			ByteBuffer buffer = ByteBuffer.allocateDirect(MAX_PACKET_BYTES);

			while (running) {
				try {
					buffer.clear();
					SocketAddress remote = channel.receive(buffer);
					if (remote == null) {
						continue;
					}

					buffer.flip();
					String json = StandardCharsets.UTF_8.decode(buffer).toString();
					packetCount++;
					packetCountPub.set(packetCount);
					rawJsonPub.set(json);

					lastPacketTimestamp = Timer.getFPGATimestamp();
					lastPacketTimestampPub.set(lastPacketTimestamp);
					packetAgePub.set(0.0);
					stalePub.set(false);

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
						pubs.apriltagsJsonPub.set((apriltags.isMissingNode() || apriltags.isNull()) ? "[]" : apriltags.toString());
						pubs.objectsJsonPub.set((objects.isMissingNode() || objects.isNull()) ? "[]" : objects.toString());

						JsonNode robotPose = root.path("robot_pose");
						boolean hasPose = !robotPose.isMissingNode() && !robotPose.isNull();
						pubs.hasPosePub.set(hasPose);
						hasPosePub.set(hasPose);

						if (hasPose) {
							double robotX = robotPose.path("x").asDouble(0.0);
							double robotY = robotPose.path("y").asDouble(0.0);
							int tagsUsed = robotPose.path("tags_used").asInt(0);
							double floorErr = robotPose.path("floor_z_error_avg").asDouble(0.0);

							pubs.robotXPub.set(robotX);
							pubs.robotYPub.set(robotY);
							pubs.tagsUsedPub.set(tagsUsed);
							pubs.floorErrPub.set(floorErr);

							robotXPub.set(robotX);
							robotYPub.set(robotY);
							tagsUsedPub.set(tagsUsed);
							floorErrPub.set(floorErr);
						} else {
							clearPose(pubs);
						}

						pubs.packetCount++;
						pubs.packetCountPub.set(pubs.packetCount);
					} catch (Exception parseErr) {
						parseErrorCount++;
						parseErrorCountPub.set(parseErrorCount);
					}
				} catch (AsynchronousCloseException closed) {
					break;
				} catch (IOException ioErr) {
					if (!running) {
						break;
					}
					ioErrorCount++;
					ioErrorCountPub.set(ioErrorCount);
				}
			}

			relayRunningPub.set(false);
		}

		private void clearPose(CameraPublishers pubs) {
			pubs.hasPosePub.set(false);
			pubs.robotXPub.set(0.0);
			pubs.robotYPub.set(0.0);
			pubs.tagsUsedPub.set(0);
			pubs.floorErrPub.set(0.0);

			hasPosePub.set(false);
			robotXPub.set(0.0);
			robotYPub.set(0.0);
			tagsUsedPub.set(0);
			floorErrPub.set(0.0);
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
