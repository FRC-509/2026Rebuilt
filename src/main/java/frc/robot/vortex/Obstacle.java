package frc.robot.vortex;

import frc.robot.Constants;

public class Obstacle extends PVector {

    private double radius;
    private double b;

    // TODO: implement future predictions next

    public Obstacle(double x, double y, double radius) {
        super(x, y);
        this.radius = radius;
        this.b = this.getEffectiveRadius() * Math.PI / 2;
    }

    // Add robot radius to obstacle radius in order to treat the robot as a point during pathing
    public double getEffectiveRadius() {
        return radius + Constants.Chassis.kRobotWidth + Constants.PathGeneration.kSafePathTolerance;
    }

    public double cosineFieldFunction(PVector position) {
        double dist = position.distanceTo(this);
        return dist < this.getEffectiveRadius() 
            ? b/2.0d * Math.cos(Math.PI * dist / b) // + self.b/2.0d
            : 0;
    }

    public PVector cosineGradientFunction(PVector position) {
        double dist = position.distanceTo(this);
        if (dist > this.getEffectiveRadius() || dist == 0) return kOrigin;
        return position.minus(this).norm().scalar(
            Math.PI/2.0d * Math.sin(Math.PI * dist * b) * Constants.PathGeneration.kAdjustRate
        );
    }

    public double gaussianFieldFunction(PVector position) {
        double dist = position.distanceTo(this);
        return dist < this.getEffectiveRadius() 
            ? b * Math.exp(-Math.pow(dist/getEffectiveRadius(),2))
            : 0;
    }

    public PVector gaussianGradientFunction(PVector position) {
        double dist = position.distanceTo(this);
        PVector vdiff = position.minus(this);

        if (dist > this.getEffectiveRadius() || dist == 0) return kOrigin;
        return new PVector(
            2.0d * Math.PI * vdiff.x / getEffectiveRadius() * Math.exp(-Math.pow(vdiff.x/getEffectiveRadius(),2.0d)),
            2.0d * Math.PI * vdiff.y / getEffectiveRadius() * Math.exp(-Math.pow(vdiff.y/getEffectiveRadius(),2.0d))
        );
    }

}
