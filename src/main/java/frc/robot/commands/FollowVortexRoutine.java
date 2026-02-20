package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vortex;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.vortex.FollowPath;
import frc.robot.vortex.PathPoint;

public class FollowVortexRoutine extends ParallelCommandGroup {
    
    private class Event {
        public String name;
        public double t;
    }

    private class StormChaserPoint {
        public int id;
        public double x;
        public double y;
        public double rotation;

        public PathPoint toPathPoint() {
            PathPoint p = new PathPoint(x,y,0);
            p.rotationTarget = rotation;
            return p;
        }
    }

    private class StormChaserPath {
        public ArrayList<StormChaserPoint> points;
        public ArrayList<Event> events;

        private StormChaserPath(){}
        
        public static StormChaserPath pathFromFile(String filePath) {
            try {
                return new ObjectMapper().readValue(new File(filePath), StormChaserPath.class);
            } catch (IOException exception) {
                DriverStation.reportError("Unhandled IO Exception when loading '"+filePath+"'.", exception.getStackTrace());
                return null;
            }
        }
    }

    private SwerveDrive swerve; 
    private Hopper hopper; 
    private Turret leftTurret;
    private Turret rightTurret;

    public FollowVortexRoutine(String name, double exitVelocity, double maxVelocity, Vortex vortex, SwerveDrive swerve, Hopper hopper, Turret leftTurret, Turret rightTurret) {
        StormChaserPath pathFile = StormChaserPath.pathFromFile(name);
        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>(pathFile.points.size());
        for (int i = 0; i < pathPoints.size(); i++) pathPoints.add(pathFile.points.get(i).toPathPoint());

        FollowPath path = new FollowPath(pathPoints, exitVelocity, maxVelocity, swerve);
        
        DoubleSupplier pathProgress = () -> path.getPathProgress(); // TODO: check if this works?
        SequentialCommandGroup[] eventCommands = new SequentialCommandGroup[pathFile.events.size()];
        for (int i = 0; i < eventCommands.length; i++) {
            Event event = pathFile.events.get(i);
            eventCommands[i] = new SequentialCommandGroup(
                Commands.waitUntil(() -> pathProgress.getAsDouble() >= event.t),
                getCommandFromEvent(event.name)
            );
        }
        
        addCommands(path);
        addCommands(eventCommands);
    }

    // for each event in vortex/events.json, return a command corosponding to the action
    private Command getCommandFromEvent(String event) {
        switch (event) {
            case "intake":
                return new HopperDefaultCommand(hopper,
                true,
                false,
                false,
                false,
                () -> leftTurret.isAbleToShoot(), () -> rightTurret.isAbleToShoot());

            case "index":
                return new HopperDefaultCommand(hopper,
                false,
                true,
                false,
                false,
                () -> leftTurret.isAbleToShoot(), () -> rightTurret.isAbleToShoot());

            case "intake_and_index":
                return new HopperDefaultCommand(hopper,
                true,
                true,
                false,
                false,
                () -> leftTurret.isAbleToShoot(), () -> rightTurret.isAbleToShoot());

            case "passive":
                return new HopperDefaultCommand(hopper,
                false,
                false,
                false,
                false,
                () -> leftTurret.isAbleToShoot(), () -> rightTurret.isAbleToShoot());

            case "climbL1":
                return Commands.none(); // no climb :(

            default:
                return Commands.runOnce(() -> DriverStation.reportError("Did not handle event '" + event + "'. This is not a good thing", null));
        }
    }


    private ParallelCommandGroup endAuto() {
        return new ParallelCommandGroup(
			Commands.runOnce(() -> swerve.stopModules(), swerve),
			new HopperDefaultCommand(
                hopper, 
                false, 
                false,
                false, 
                false,
                () -> false, 
                () -> false 
            ) // make leds turn green
        );
    }
    
}
