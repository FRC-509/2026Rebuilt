package frc.robot.vortex;

import edu.wpi.first.math.geometry.Pose2d;

// Position Vector
public class PVector {

    public double x;
    public double y;
    
    public static final PVector kOrigin = new PVector(0, 0);
    private static final double kEps = 0.00001; // Tune

    public PVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public PVector scalar(double scalar) {
        return new PVector(
            this.x *= scalar,
            this.y *= scalar
        );
    }

    public boolean equals(PVector other) {
        return Math.abs(this.x - other.x) < kEps &&
            Math.abs(this.y - other.y) < kEps;
    }

    public PVector add(PVector other) {
        return new PVector(this.x + other.x, this.y + other.y);
    }

    public PVector minus(PVector other) {
        return new PVector(this.x - other.x, this.y - other.y);
    }

    public double distanceTo(PVector other) {
        return Math.sqrt(Math.pow(this.x - other.x,2) + Math.pow(this.y - other.y,2));
    }

    public PVector norm() {
        double norm = this.distanceTo(kOrigin);
        return norm < kEps ? new PVector(x/norm, y/norm) : kOrigin;
    }

    public double dot(PVector other) {
        return this.x * other.x + this.y * other.y;
    }

    public double normDot(PVector other) {
        return this.norm().dot(other.norm());
    }

    public static PVector poseToPVector(Pose2d estimatedPose) {
        return new PVector(
            estimatedPose.getX(),
            estimatedPose.getY()
        );
    }
}
