package frc.robot.path_generation;

public class PathPoint extends PVector {
    public double height;
    public double rotationTarget;
    
    public PathPoint(double x, double y, double height) {
        super(x, y);
        this.height = height;
        this.rotationTarget = 0d;
    }

    public PathPoint(PVector pVector) {
        super(pVector.x, pVector.y);
        this.height = 0;
        this.rotationTarget = 0d;
    }

}
