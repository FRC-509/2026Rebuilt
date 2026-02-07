package frc.robot.vortex;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

public class FollowPath extends Command {

    private SwerveDrive swerveDrive;
    private Pair<ArrayList<PathPoint>,Boolean> path;

    private boolean pathIsValid;
    private boolean forceValid;
    private boolean isFinished;
    private boolean finishOnceDone;

    private Timer elapsedTimer;
    private double lastUpdateTimestamp;

    private double pathProgress;
    private boolean followPath;
    
    private Pose2d theoreticalPose;
    private Pose2d targetPose;
    private double targetVelocity;

    public FollowPath(ArrayList<PathPoint> path, SwerveDrive swerve) {
        this.isFinished = false;
        this.elapsedTimer = new Timer();

        this.targetVelocity = Constants.Chassis.kMaxSpeed * 0.2; // Max Speed Wanted to reach, make clamp of distanceTo start/finish

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.path = PathGeneration.generatePath(null, null, swerveDrive);
        pathIsValid = this.path.getSecond().booleanValue() || forceValid;
        if (!pathIsValid) {
            elapsedTimer.restart();
            followPath = false;
        }
    }

    @Override
    public void execute() {

        // Path completion behavior
        if (pathProgress >= 1.0) {
            swerveDrive.setChassisSpeeds(new ChassisSpeeds(0,0,0));
            isFinished = true;
            return;
        }

        // calculate latency
        if (pathProgress != 0.0d) { // no latency on first pass
            pathProgress += elapsedTimer.get() - lastUpdateTimestamp;
            lastUpdateTimestamp = elapsedTimer.get();
        }

        // calculate predictive secant step along spline path through path points 
        double dt = findStepForLength(pathProgress, targetVelocity, 0,1);
        PVector currentPosition = catmullRomSpline(pathProgress);
        PVector targetPos = catmullRomSpline(pathProgress + dt);
        
        // get actual 
        PVector deltaMove = targetPos.minus(currentPosition);
        ChassisSpeeds currentChassisSpeeds = swerveDrive.getChassisSpeeds();
        PVector deltaV = deltaMove.norm().scalar(
            new PVector(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond).distanceTo(PVector.kOrigin));
        
        swerveDrive.drive(
            new Translation2d(deltaV.x, deltaV.y),
            0d,
            true, 
            false);
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
        if (path.getFirst().size() < 4) {
            if (path.getFirst().isEmpty()) return PVector.kOrigin;
            else return path.getFirst().get(0);
        }

        int numSegments = path.getFirst().size() - 3;
        double tSegment = t*((double)numSegments);
        int idxSegment = ((int)tSegment);
        double tLocal = tSegment - idxSegment;

        idxSegment = Math.min(idxSegment, numSegments-1);

        PVector p0 = path.getFirst().get(idxSegment);
        PVector p1 = path.getFirst().get(idxSegment+1);
        PVector p2 = path.getFirst().get(idxSegment+2);
        PVector p3 = path.getFirst().get(idxSegment+3);

        return catmullRomPoint(tLocal, p0, p1, p2, p3);
    }

    public Pair<PVector,PVector> getBetweenPoints(double t) {
        if (path.getFirst().size() < 4) {
            if (path.getFirst().isEmpty()) return new Pair<PVector,PVector>(PVector.kOrigin,PVector.kOrigin);
            else return new Pair<PVector,PVector>(path.getFirst().get(0),path.getFirst().get(1));
        }

        int numSegments = path.getFirst().size() - 3;
        double tSegment = t*((double)numSegments);
        int idxSegment = ((int)tSegment);

        idxSegment = Math.min(idxSegment, numSegments-1);

        return new Pair<PVector,PVector>(
            path.getFirst().get(idxSegment+1), 
            path.getFirst().get(idxSegment+2)
        );
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
    
    @Override
    public void end(boolean interrupted) {
        swerveDrive.setChassisSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public double getPathProgress() {
        return pathProgress;
    }

}
