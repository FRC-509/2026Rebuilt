package frc.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
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

    public record CameraState(
        int schemaVersion,
        double fps,
        int cameraIndex,
        boolean hasPose,
        double robotX,
        double robotY,
        int tagsUsed,
        double floorZErrorAvg,
        String apriltagsJson,
        String objectsJson,
        int packetCount
    ) {
        public static final CameraState EMPTY = new CameraState(0, 0.0, -1, false, 0.0, 0.0, 0, 0.0, "[]", "[]", 0);
    }

    public record RelayState(
        String rawJson,
        String sourceIp,
        int sourcePort,
        int packetCount,
        int parseErrorCount,
        int ioErrorCount,
        double lastPacketTimestamp,
        boolean relayRunning,
        boolean hasPose,
        double robotX,
        double robotY,
        int tagsUsed,
        double floorZErrorAvg,
        CameraState[] cameras
    ) {
        public static RelayState empty() {
            return new RelayState("", "", 0, 0, 0, 0, Double.NaN, false, false, 0.0, 0.0, 0, 0.0,
                new CameraState[] {CameraState.EMPTY, CameraState.EMPTY});
        }
    }

    private final Receiver receiver;

    public JetsonUdpRelay(String name, int port) throws IOException {
        receiver = new Receiver(name, port);
    }

    public void poll() {
        // Receive/parsing runs on the worker thread.
    }

    public void close() {
        receiver.close();
    }

    public RelayState getState() {
        return copyState(receiver.getState());
    }

    public boolean isRunning() {
        return getState().relayRunning();
    }

    public boolean isStale() {
        RelayState state = getState();
        return Double.isNaN(state.lastPacketTimestamp())
            || (Timer.getFPGATimestamp() - state.lastPacketTimestamp()) > STALE_PACKET_SECONDS;
    }

    public double getPacketAgeSeconds() {
        RelayState state = getState();
        if (Double.isNaN(state.lastPacketTimestamp())) {
            return Double.NaN;
        }
        return Timer.getFPGATimestamp() - state.lastPacketTimestamp();
    }

    public boolean hasPose() {
        if (isStale()) {
            return false;
        }

        for (CameraState camera : getState().cameras()) {
            if (camera.hasPose()) {
                return true;
            }
        }
        return false;
    }

    public double getRobotX() {
        return averageCameraValue(CameraState::robotX, getState());
    }

    public double getRobotY() {
        return averageCameraValue(CameraState::robotY, getState());
    }

    public int getTagsUsed() {
        RelayState state = getState();
        int maxTagsUsed = 0;
        for (CameraState camera : state.cameras()) {
            if (camera.hasPose()) {
                maxTagsUsed = Math.max(maxTagsUsed, camera.tagsUsed());
            }
        }
        return maxTagsUsed;
    }

    public double getFloorZErrorAvg() {
        return averageCameraValue(CameraState::floorZErrorAvg, getState());
    }

    public double getLastPacketTimestamp() {
        return getState().lastPacketTimestamp();
    }

    public CameraState getCameraState(int slot) {
        CameraState[] cameras = getState().cameras();
        if (slot < 0 || slot >= cameras.length) {
            return CameraState.EMPTY;
        }
        return cameras[slot];
    }

    private static RelayState copyState(RelayState state) {
        CameraState[] cameras = new CameraState[state.cameras().length];
        System.arraycopy(state.cameras(), 0, cameras, 0, state.cameras().length);
        return new RelayState(
            state.rawJson(),
            state.sourceIp(),
            state.sourcePort(),
            state.packetCount(),
            state.parseErrorCount(),
            state.ioErrorCount(),
            state.lastPacketTimestamp(),
            state.relayRunning(),
            state.hasPose(),
            state.robotX(),
            state.robotY(),
            state.tagsUsed(),
            state.floorZErrorAvg(),
            cameras);
    }

    private static double averageCameraValue(CameraValueExtractor extractor, RelayState state) {
        double sum = 0.0;
        int count = 0;
        for (CameraState camera : state.cameras()) {
            if (camera.hasPose()) {
                sum += extractor.extract(camera);
                count++;
            }
        }

        if (count == 0) {
            return 0.0;
        }
        return sum / count;
    }

    @FunctionalInterface
    private interface CameraValueExtractor {
        double extract(CameraState cameraState);
    }

    private static final class Receiver {
        private final DatagramChannel channel;
        private final Thread worker;
        private final Map<Integer, Integer> sourceIndexToSlot = new HashMap<>();

        private volatile boolean running = true;
        private volatile RelayState state = RelayState.empty();
        private int nextSlot = 0;

        Receiver(String name, int port) throws IOException {
            this.channel = DatagramChannel.open();
            this.channel.configureBlocking(true);
            this.channel.bind(new InetSocketAddress(port));

            RelayState initialState = RelayState.empty();
            state = new RelayState(
                initialState.rawJson(),
                initialState.sourceIp(),
                initialState.sourcePort(),
                initialState.packetCount(),
                initialState.parseErrorCount(),
                initialState.ioErrorCount(),
                initialState.lastPacketTimestamp(),
                true,
                initialState.hasPose(),
                initialState.robotX(),
                initialState.robotY(),
                initialState.tagsUsed(),
                initialState.floorZErrorAvg(),
                initialState.cameras());

            worker = new Thread(this::run, "JetsonUdpRelay-" + name + "-" + port);
            worker.setDaemon(true);
            worker.start();
        }

        RelayState getState() {
            return state;
        }

        void close() {
            running = false;
            RelayState current = state;
            state = new RelayState(
                current.rawJson(),
                current.sourceIp(),
                current.sourcePort(),
                current.packetCount(),
                current.parseErrorCount(),
                current.ioErrorCount(),
                current.lastPacketTimestamp(),
                false,
                current.hasPose(),
                current.robotX(),
                current.robotY(),
                current.tagsUsed(),
                current.floorZErrorAvg(),
                current.cameras());

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
                    double packetTimestamp = Timer.getFPGATimestamp();

                    RelayState current = state;
                    CameraState[] cameras = copyCameras(current.cameras());
                    String sourceIp = current.sourceIp();
                    int sourcePort = current.sourcePort();
                    if (remote instanceof InetSocketAddress addr) {
                        sourceIp = addr.getAddress().getHostAddress();
                        sourcePort = addr.getPort();
                    }

                    state = new RelayState(
                        json,
                        sourceIp,
                        sourcePort,
                        current.packetCount() + 1,
                        current.parseErrorCount(),
                        current.ioErrorCount(),
                        packetTimestamp,
                        true,
                        current.hasPose(),
                        current.robotX(),
                        current.robotY(),
                        current.tagsUsed(),
                        current.floorZErrorAvg(),
                        cameras);

                    try {
                        JsonNode root = MAPPER.readTree(json);
                        int sourceCameraIndex = root.path("camera_index").asInt(-1);
                        if (sourceCameraIndex < 0) {
                            bumpParseError();
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

                        CameraState previousCamera = cameras[slot];
                        JsonNode apriltags = root.path("apriltags");
                        JsonNode objects = root.path("objects");
                        JsonNode robotPose = root.path("robot_pose");
                        boolean hasPose = !robotPose.isMissingNode() && !robotPose.isNull();

                        CameraState updatedCamera = new CameraState(
                            root.path("schema_version").asInt(0),
                            root.path("fps").asDouble(0.0),
                            sourceCameraIndex,
                            hasPose,
                            hasPose ? robotPose.path("x").asDouble(0.0) : 0.0,
                            hasPose ? robotPose.path("y").asDouble(0.0) : 0.0,
                            hasPose ? robotPose.path("tags_used").asInt(0) : 0,
                            hasPose ? robotPose.path("floor_z_error_avg").asDouble(0.0) : 0.0,
                            (apriltags.isMissingNode() || apriltags.isNull()) ? "[]" : apriltags.toString(),
                            (objects.isMissingNode() || objects.isNull()) ? "[]" : objects.toString(),
                            previousCamera.packetCount() + 1);
                        cameras[slot] = updatedCamera;

                        RelayState latest = state;
                        boolean aggregateHasPose = false;
                        double robotXSum = 0.0;
                        double robotYSum = 0.0;
                        double floorErrorSum = 0.0;
                        int poseCameraCount = 0;
                        int maxTagsUsed = 0;
                        for (CameraState camera : cameras) {
                            if (!camera.hasPose()) {
                                continue;
                            }
                            aggregateHasPose = true;
                            robotXSum += camera.robotX();
                            robotYSum += camera.robotY();
                            floorErrorSum += camera.floorZErrorAvg();
                            maxTagsUsed = Math.max(maxTagsUsed, camera.tagsUsed());
                            poseCameraCount++;
                        }

                        state = new RelayState(
                            latest.rawJson(),
                            latest.sourceIp(),
                            latest.sourcePort(),
                            latest.packetCount(),
                            latest.parseErrorCount(),
                            latest.ioErrorCount(),
                            latest.lastPacketTimestamp(),
                            true,
                            aggregateHasPose,
                            poseCameraCount == 0 ? 0.0 : robotXSum / poseCameraCount,
                            poseCameraCount == 0 ? 0.0 : robotYSum / poseCameraCount,
                            maxTagsUsed,
                            poseCameraCount == 0 ? 0.0 : floorErrorSum / poseCameraCount,
                            copyCameras(cameras));
                    } catch (Exception parseErr) {
                        bumpParseError();
                    }
                } catch (AsynchronousCloseException closed) {
                    break;
                } catch (IOException ioErr) {
                    if (!running) {
                        break;
                    }
                    bumpIoError();
                }
            }

            RelayState current = state;
            state = new RelayState(
                current.rawJson(),
                current.sourceIp(),
                current.sourcePort(),
                current.packetCount(),
                current.parseErrorCount(),
                current.ioErrorCount(),
                current.lastPacketTimestamp(),
                false,
                current.hasPose(),
                current.robotX(),
                current.robotY(),
                current.tagsUsed(),
                current.floorZErrorAvg(),
                current.cameras());
        }

        private void bumpParseError() {
            RelayState current = state;
            state = new RelayState(
                current.rawJson(),
                current.sourceIp(),
                current.sourcePort(),
                current.packetCount(),
                current.parseErrorCount() + 1,
                current.ioErrorCount(),
                current.lastPacketTimestamp(),
                current.relayRunning(),
                current.hasPose(),
                current.robotX(),
                current.robotY(),
                current.tagsUsed(),
                current.floorZErrorAvg(),
                current.cameras());
        }

        private void bumpIoError() {
            RelayState current = state;
            state = new RelayState(
                current.rawJson(),
                current.sourceIp(),
                current.sourcePort(),
                current.packetCount(),
                current.parseErrorCount(),
                current.ioErrorCount() + 1,
                current.lastPacketTimestamp(),
                current.relayRunning(),
                current.hasPose(),
                current.robotX(),
                current.robotY(),
                current.tagsUsed(),
                current.floorZErrorAvg(),
                current.cameras());
        }

        private static CameraState[] copyCameras(CameraState[] original) {
            CameraState[] copy = new CameraState[original.length];
            System.arraycopy(original, 0, copy, 0, original.length);
            return copy;
        }
    }
}
