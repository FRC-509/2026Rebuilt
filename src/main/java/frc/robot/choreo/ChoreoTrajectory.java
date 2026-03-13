package frc.robot.choreo;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class ChoreoTrajectory {
    private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();

    private final String name;
    private final List<Sample> samples;

    private ChoreoTrajectory(String name, List<Sample> samples) {
        this.name = name;
        this.samples = samples;
    }

    public static ChoreoTrajectory load(String name) {
        Path deployPath = Filesystem.getDeployDirectory().toPath()
            .resolve("choreo_routines")
            .resolve(name + ".traj");
        File trajectoryFile = deployPath.toFile();
        if (!trajectoryFile.exists()) {
            trajectoryFile = Path.of("choreo_routines", name + ".traj").toFile();
        }

        try {
            TrajectoryFile file = OBJECT_MAPPER.readValue(trajectoryFile, TrajectoryFile.class);
            if (file == null || file.trajectory == null || file.trajectory.samples == null || file.trajectory.samples.isEmpty()) {
                DriverStation.reportError("Choreo trajectory '" + name + "' is empty.", false);
                return new ChoreoTrajectory(name, List.of());
            }
            return new ChoreoTrajectory(name, file.trajectory.samples);
        } catch (IOException exception) {
            DriverStation.reportError("Failed to load choreo trajectory '" + name + "'.", exception.getStackTrace());
            return new ChoreoTrajectory(name, List.of());
        }
    }

    public boolean isEmpty() {
        return samples.isEmpty();
    }

    public String getName() {
        return name;
    }

    public Pose2d getInitialPose() {
        Sample first = getInitialSample();
        return new Pose2d(first.x, first.y, Rotation2d.fromRadians(first.heading));
    }

    public Sample getInitialSample() {
        return isEmpty() ? Sample.ZERO : samples.get(0);
    }

    public double getTotalTimeSeconds() {
        return isEmpty() ? 0.0 : samples.get(samples.size() - 1).t;
    }

    public Sample sample(double timeSeconds) {
        if (isEmpty()) {
            return Sample.ZERO;
        }
        if (timeSeconds <= samples.get(0).t) {
            return samples.get(0);
        }

        Sample last = samples.get(samples.size() - 1);
        if (timeSeconds >= last.t) {
            return last;
        }

        for (int i = 1; i < samples.size(); i++) {
            Sample upper = samples.get(i);
            if (timeSeconds > upper.t) {
                continue;
            }

            Sample lower = samples.get(i - 1);
            double blend = MathUtil.inverseInterpolate(lower.t, upper.t, timeSeconds);
            return Sample.interpolate(lower, upper, blend);
        }

        return last;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    private static class TrajectoryFile {
        public TrajectorySection trajectory;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    private static class TrajectorySection {
        public List<Sample> samples;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Sample {
        public static final Sample ZERO = new Sample();

        public double t;
        public double x;
        public double y;
        public double heading;
        public double vx;
        public double vy;
        public double omega;

        public Pose2d pose() {
            return new Pose2d(x, y, Rotation2d.fromRadians(heading));
        }

        public static Sample interpolate(Sample a, Sample b, double blend) {
            Sample sample = new Sample();
            sample.t = MathUtil.interpolate(a.t, b.t, blend);
            sample.x = MathUtil.interpolate(a.x, b.x, blend);
            sample.y = MathUtil.interpolate(a.y, b.y, blend);
            sample.heading = Rotation2d.fromRadians(a.heading)
                .interpolate(Rotation2d.fromRadians(b.heading), blend)
                .getRadians();
            sample.vx = MathUtil.interpolate(a.vx, b.vx, blend);
            sample.vy = MathUtil.interpolate(a.vy, b.vy, blend);
            sample.omega = MathUtil.interpolate(a.omega, b.omega, blend);
            return sample;
        }
    }
}
