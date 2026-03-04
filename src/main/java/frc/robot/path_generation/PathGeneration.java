package frc.robot.path_generation;

import java.util.ArrayList;
import java.util.function.Predicate;

import edu.wpi.first.math.Pair;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

public class PathGeneration {

    private static final int kDefaultPathSegmentCount = 200;
    private static final double kMinAdjust = 0.001;
    private static final double kMinSafeDistance = 0.01; // m
    private static final int kMaxIterations = 2000;
    private static final double kPathOptimizationThreshold = 0.001;

    /**
     * 
     * @param initialPosition
     * @param targetPosition
     * @param segmentCount
     * @param obstacles
     * @return 
     */
    public static Pair<ArrayList<PathPoint>,Boolean> generatePath(PVector initialPosition, PVector targetPosition, int segmentCount, Obstacle[] obstacles, SwerveDrive swerve) {
        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();
        pathPoints.ensureCapacity(segmentCount + 2);
        pathPoints.add(new PathPoint(initialPosition));

        PVector diff = targetPosition.minus(initialPosition);
        for (int i = 1; i <= segmentCount; i++) {
            PVector prevPoint = pathPoints.get(i-1);
            PathPoint pathPoint = new PathPoint(
                prevPoint.x + diff.x, 
                prevPoint.y + diff.y,
                0.0d);
            for (Obstacle obstacle : obstacles) pathPoint.height += obstacle.cosineFieldFunction(pathPoint);
            pathPoints.add(pathPoint);
        }
        pathPoints.add(new PathPoint(targetPosition));

        cleanPath(pathPoints, segmentCount);
        optimizePath(pathPoints, obstacles);
        boolean isPathValid = validatePath(pathPoints);
        removePointsOfCurvature(pathPoints);

        return new Pair<ArrayList<PathPoint>,Boolean>(pathPoints, isPathValid);
    }

    public static Pair<ArrayList<PathPoint>,Boolean> generatePath(PVector initialPosition, PVector targetPosition, Obstacle[] obstacles, SwerveDrive swerve) {
        return generatePath(PVector.poseToPVector(swerve.getEstimatedPose()), targetPosition, kDefaultPathSegmentCount, obstacles, swerve);
    }

    public static Pair<ArrayList<PathPoint>,Boolean> generatePath(PVector targetPosition, int segmentCount, Obstacle[] obstacles, SwerveDrive swerve) {
        return generatePath(PVector.poseToPVector(swerve.getEstimatedPose()), targetPosition, segmentCount, obstacles, swerve);
    }

    public static Pair<ArrayList<PathPoint>,Boolean> generatePath(PVector targetPosition, Obstacle[] obstacles, SwerveDrive swerve) {
        return generatePath(PVector.poseToPVector(swerve.getEstimatedPose()), targetPosition, kDefaultPathSegmentCount, obstacles, swerve);
    }

    private static void cleanPath(ArrayList<PathPoint> pathPoints, double stepDistance) {
        double threshold = stepDistance * 1.3;
        int i = 1;
        
        while (i < pathPoints.size() - 1) {
            if (pathPoints.size() <= 12) break;
            if (i == 0) { i = 1; continue; }
            
            if (pathPoints.get(i).distanceTo(pathPoints.get(i-1)) < threshold) pathPoints.remove(i);
            else i++;
        }
    }

    private static boolean optimizePath(ArrayList<PathPoint> pathPoints, Obstacle[] obstacles) {
        if (pathPoints.size() <= 2) return true;
        
        int iterations = 0;
        while (iterations < kMaxIterations && !optimizePathSingleIteration(pathPoints, obstacles)) iterations++;
        if (isPathOptimized(pathPoints, obstacles)) {
            for (PathPoint point : pathPoints) point.height = 0.0d;
            return true;
        }
        return false;
    }

    private static boolean optimizePathSingleIteration(ArrayList<PathPoint> pathPoints, Obstacle[] obstacles) {
        if (pathPoints.size() <= 2) return true;
        boolean allPointsOptimized = true;

        for (int i = 0; i < pathPoints.size(); i++) {
            PathPoint point = pathPoints.get(i);
            boolean tooCloseToObstacle = false;

            for (Obstacle obstacle : obstacles) {
                if (point.distanceTo(obstacle) < kMinSafeDistance) {
                    tooCloseToObstacle = true;
                    break;
                }
            }

            if (point.height > kPathOptimizationThreshold || !tooCloseToObstacle) {
                PVector totalDelta = new PVector(0, 0);

                for (Obstacle obstacle : obstacles) {
                    PVector gradient = obstacle.cosineGradientFunction(totalDelta);
                    
                    if (point.distanceTo(obstacle) < kMinSafeDistance) { // double check all logic
                        PVector diff = point.minus(obstacle);
                        if (diff.distanceTo(PVector.kOrigin) > kPathOptimizationThreshold)
                            totalDelta = totalDelta.minus(diff.norm());
                    } else {
                        totalDelta = totalDelta.minus(gradient);
                    }
                }

                // min adjust rate TODO: Can this be moved?
                if (Math.abs(totalDelta.x) < kMinAdjust && Math.abs(totalDelta.y) < kMinAdjust) {
                    if (totalDelta.x < kMinAdjust) totalDelta.x = Math.signum(totalDelta.x) * kMinAdjust;
                    if (totalDelta.y < kMinAdjust) totalDelta.y = Math.signum(totalDelta.y) * kMinAdjust; 
                }

                point.add(totalDelta);

                double height = 0.0d;
                for (Obstacle obstacle : obstacles) height += obstacle.cosineFieldFunction(point);
                point.height = height;
            }
        }

        return allPointsOptimized;
    }

    private static boolean isPathOptimized(ArrayList<PathPoint> pathPoints, Obstacle[] obstacles) {
        for (int i = 1; i < pathPoints.size(); i++) {
            if (pathPoints.get(i).height > kPathOptimizationThreshold) return false;
        } // test implementation had check to see if it is far enough away from all obst, redundant?
        return true;
    }

    private static void removePointsOfCurvature(ArrayList<PathPoint> pathPoints) {
        ArrayList<Integer> pointsOfCurvature = new ArrayList<Integer>();

        PathPoint pathStart = pathPoints.get(0);
        int i = 2;
        while (i < pathPoints.size() - 2) {
            if (i == 0) i = 2;

            PVector v1 = pathPoints.get(i-1).minus(pathStart);
            PVector v2 = pathPoints.get(i).minus(pathStart);
            PVector v3 = pathPoints.get(i+1).minus(pathStart);
            PVector vPath = pathPoints.get(pathPoints.size()-1).minus(pathStart);

            Predicate<PVector> isColinearToPath = (v) -> {
                if (v.equals(PVector.kOrigin)) return false;                
                return v.dot(vPath)/(v.distanceTo(PVector.kOrigin) * vPath.distanceTo(PVector.kOrigin)) > 0.999999; // 1-epsilon to account for floating point error
            };

            if (isColinearToPath.test(v2) && (isColinearToPath.test(v1) ^ isColinearToPath.test(v3)) && i < pathPoints.size() - 2){
                    double dist = 0.0d;
                    int removeCount = 0;
                    PVector center = pathPoints.get(i);
                    do { //TODO: double check do-while here
                        dist = pathPoints.get(i-removeCount).distanceTo(center);
                        removeCount++;
                    } while (dist < Constants.PathGeneration.kCurvaturePointRemovalRadius
                        && i + removeCount < pathPoints.size()-2
                        && i - removeCount > 1);
                for (int j = i - removeCount; j <= i+ removeCount; j++) pointsOfCurvature.add(j);
                i += removeCount + 1;
            } else i++;
        }

        for (int index = pointsOfCurvature.size()-1; index >= 0 ; index--) pathPoints.remove(pointsOfCurvature.get(index).intValue());
    }

    private static boolean validatePath(ArrayList<PathPoint> pathPoints) {
        return true; // TODO: implement once simulated correctly
    }
}
