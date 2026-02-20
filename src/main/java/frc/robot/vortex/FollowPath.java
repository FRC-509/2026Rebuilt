package frc.robot.vortex;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

public class FollowPath extends Command {

    private SwerveDrive swerve;
    private ArrayList<PathPoint> path;

    private boolean isFinished;

    private Timer elapsedTimer;
    private double lastUpdateTimestamp;

    private double pathProgress;
    private double pathLength;
    private double initialVelocity;
    private double targetVelocity;
    private double maxVelocity;
    private double usableVelocity;

    private double acclerationEndT;
    private double decelerationStartT;

    public FollowPath(ArrayList<PathPoint> path, double exitVelocity, double maxVelocity, SwerveDrive swerve) {
        this.path = path;
        this.swerve = swerve;
        this.isFinished = false;
        this.elapsedTimer = new Timer();
        this.targetVelocity = MathUtil.clamp(exitVelocity, 0.0, Constants.Chassis.kMaxSpeed);
        
        this.pathLength = 0;
        PVector prev = path.get(0), next;
        for (int i = 1; i <= Constants.PathGeneration.kLengthApproximationSegments; i++) {
            next = catmullRomSpline(i / Constants.PathGeneration.kLengthApproximationSegments);
            this.pathLength += prev.distanceTo(next);
            prev = next;
        }

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.initialVelocity = getCurrentVelocityMagnitude();
        this.acclerationEndT = 1 - (Math.pow(maxVelocity - initialVelocity, 2) / Constants.Chassis.kMaxAcceleration / pathLength);
        this.acclerationEndT = (Math.pow(maxVelocity, 2) / Constants.Chassis.kMaxDecceleration) / pathLength;
    }

    @Override
    public void execute() {

        // Path completion behavior
        if (pathProgress >= 1.0) {
            swerve.setChassisSpeeds(new ChassisSpeeds(0,0,0));
            isFinished = true;
            return;
        }

        // calculate latency
        if (pathProgress != 0.0d) { // no latency on first pass
            pathProgress += elapsedTimer.get() - lastUpdateTimestamp;
            lastUpdateTimestamp = elapsedTimer.get();
        }

        // get velocity
        usableVelocity = getCurrentVelocityMagnitude();
        //prioritize slowing down in cases where path lenght is too low and starting & stopping distances overlap
        if (pathProgress > decelerationStartT) {
            usableVelocity -= Constants.Chassis.kMaxDecceleration * 0.01; // averge velocity over 20ms
            targetVelocity = usableVelocity - Constants.Chassis.kMaxAcceleration * 0.01;
        } else if (pathProgress < acclerationEndT) {
            usableVelocity += Constants.Chassis.kMaxAcceleration * 0.01;
            targetVelocity = usableVelocity + Constants.Chassis.kMaxAcceleration * 0.01;
        } else targetVelocity = maxVelocity;
        
        // calculate predictive secant step along spline path through path points 
        double dt = findStepForLength(pathProgress, usableVelocity, 0,1);
        PVector currentPosition = catmullRomSpline(pathProgress);
        PVector targetPos = catmullRomSpline(pathProgress + dt);
        
        // get actual 
        PVector deltaMove = targetPos.minus(currentPosition);
        PVector deltaV = deltaMove.norm().scalar(targetVelocity);
        
        swerve.drive(
            new Translation2d(deltaV.x, deltaV.y),
            0d,
            true, 
            false);
        swerve.setTargetHeading(getNextPoint(pathProgress).rotationTarget);
    }

    private PVector catmullRomPoint(double t, PVector p0, PVector p1, PVector p2, PVector p3) {
        double t2 = t*t;
        double t3 = t2*t;

        double b0 = 0.5 * (-t3 + 2.0*t2 - t);
        double b1 = 0.5 * (3.0*t3 - 5.0*t2 + 2.0);
        double b2 = 0.5 * (-3.0*t3 + 4.0*t2 + t);
        double b3 = 0.5 * (t3 - t2);
        
        return new PVector(
            b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
            b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y
        );
    }

    // returns the x and y coordinates for a point on a catmull rom spline at progress t 0-1
    public PVector catmullRomSpline(double t) {
        if (path.size() < 4) {
            if (path.isEmpty()) return PVector.kOrigin;
            else return path.get(0);
        }

        int numSegments = path.size() - 3;
        double tSegment = t*((double)numSegments);
        int idxSegment = ((int)tSegment);
        double tLocal = tSegment - idxSegment;

        idxSegment = Math.min(idxSegment, numSegments-1);

        PVector p0 = path.get(idxSegment);
        PVector p1 = path.get(idxSegment+1);
        PVector p2 = path.get(idxSegment+2);
        PVector p3 = path.get(idxSegment+3);

        return catmullRomPoint(tLocal, p0, p1, p2, p3);
    }

    public PathPoint getNextPoint(double t) {
        if (path.size() < 4) {
            if (path.isEmpty()) return (PathPoint) PVector.kOrigin;
            else return path.get(0);
        }

        int numSegments = path.size() - 3;
        double tSegment = t*((double)numSegments);
        int idxSegment = ((int)tSegment);

        idxSegment = Math.min(idxSegment, numSegments-1);

        return path.get(idxSegment+2);
    }

    
    private double findStepForLength(double t, double targetLength, double minStep, double maxStep) {
        double GOLDEN_RATIO_COMP = 0.38196601125;

        double currentStep = minStep + GOLDEN_RATIO_COMP * (maxStep - minStep);
        double previousBestStep = currentStep;
        double secondPreviousBestStep = currentStep;

        double currentError = Math.abs(catmullRomSpline(t + currentStep).minus(catmullRomSpline(t)).distanceTo(new PVector(0, 0)) - targetLength);
        double previousError = currentError;
        double secondPreviousError = currentError;

        double proposedStepChange = 0;
        double lastStepChange = 0;

        for (int iteration = 0; iteration < 30; iteration++) {
            double intervalMidpoint = 0.5 * (minStep + maxStep);

            double parabolaNumerator =
                (currentStep - previousBestStep) * (currentError - secondPreviousError)
                - (currentStep - secondPreviousBestStep) * (currentError - previousError);

            double parabolaDenominator =
                2 * ((currentStep - previousBestStep) * (currentError - secondPreviousError)
                - (currentStep - secondPreviousBestStep) * (currentError - previousError));

            if (parabolaDenominator != 0 && Math.abs(parabolaNumerator) < Math.abs(0.5 * parabolaDenominator * lastStepChange)) {
                proposedStepChange = parabolaNumerator / parabolaDenominator;
            } else {
                lastStepChange = (currentStep < intervalMidpoint)
                    ? maxStep - currentStep
                    : minStep - currentStep;
                proposedStepChange = GOLDEN_RATIO_COMP * lastStepChange;
            }

            double candidateStep = currentStep + proposedStepChange;
            double candidateError = Math.abs(catmullRomSpline(t + candidateStep).minus(catmullRomSpline(t)).distanceTo(new PVector(0, 0)) - targetLength);

            if (candidateError < currentError) {
                if (candidateStep < currentStep) maxStep = currentStep;
                else minStep = currentStep;

                secondPreviousBestStep = previousBestStep;
                secondPreviousError = previousError;
                previousBestStep = currentStep;
                previousError = currentError;
                currentStep = candidateStep;
                currentError = candidateError;
            } else {
                if (candidateStep < currentStep) minStep = candidateStep;
                else maxStep = candidateStep;

                if (candidateError < previousError) {
                    secondPreviousBestStep = previousBestStep;
                    secondPreviousError = previousError;
                    previousBestStep = candidateStep;
                    previousError = candidateError;
                }
            }
        }
        return currentStep;
    }

    private double getCurrentVelocityMagnitude() {
        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        return Math.sqrt(
            speeds.vxMetersPerSecond*speeds.vxMetersPerSecond 
            + speeds.vyMetersPerSecond*speeds.vyMetersPerSecond);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public double getPathProgress() {
        return pathProgress;
    }

}
