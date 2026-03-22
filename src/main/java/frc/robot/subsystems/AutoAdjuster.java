package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PigeonWrapper;
import java.util.List;

public class AutoAdjuster extends SubsystemBase {
    private final String limeLightName = "limelight-intake";
    private final NetworkTable limeLightTable;
    private final PigeonWrapper pigeon = new PigeonWrapper(0);
    private final SwerveDrive swerve;

    private class RetroTarget {
        public double ta;
        public double tx;
        public double ty;

        public RetroTarget(double ta, double tx, double ty) {
            this.ta = ta;
            this.tx = tx;
            this.ty = ty;
        }

        public static RetroTarget getLargestRetroTarget(List<RetroTarget> targets) {
            RetroTarget largest = null;
            double maxArea = 0.0; 
            for (RetroTarget target : targets) {
                if (target.ta > maxArea) {
                    maxArea = target.ta;
                    largest = target;
                }
            }
            return largest;
        }
    }
    public AutoAdjuster(){
        limeLightTable = NetworkTableInstance.getDefault().getTable(limeLightName);
        this.swerve = new SwerveDrive(pigeon);
    }

    // public double getDistance(){
    //     return Constants.Vision.kCameraHeight / Math.cos(LimelightHelpers.getTXNC(limeLightName));
    // }

    public void adjustSwerveDrive(List<RetroTarget> targets){
        
        RetroTarget largest = RetroTarget.getLargestRetroTarget(targets);
        if (largest != null) {
        double rotation = Constants.PIDConstants.Drive.kSteerAngleP * LimelightHelpers.getTX(limeLightName);
        swerve.runOnce(() -> new DefaultDriveCommand(swerve, 0.0, 0.0, rotation, true));
        }
    }

    @Override
    public void periodic(){
        boolean hasTarget = LimelightHelpers.getTV(limeLightName);
        LimelightHelpers.setLEDMode_PipelineControl(limeLightName);
        LimelightHelpers.setLEDMode_ForceOn(limeLightName);
                //SmartDashboard.putNumber
        if(hasTarget){
            double xOffset = LimelightHelpers.getTX(limeLightName);
            double yOffset = LimelightHelpers.getTY(limeLightName);
            double area = LimelightHelpers.getTA(limeLightName);
            SmartDashboard.putBoolean("Has Target", hasTarget);
            SmartDashboard.putNumber("X Offset", xOffset);
            SmartDashboard.putNumber("Y Offset", yOffset);
            SmartDashboard.putNumber("Area", area);
            LimelightHelpers.setLEDMode_ForceOff(limeLightName);
            
          //  SmartDashboard.putstr(limeLightName, limeLightName)("LimeLight", LimelightHelpers.getJSONDump(limeLightName));
        } else {
            SmartDashboard.putBoolean("Has Target", false);
            LimelightHelpers.setLEDMode_ForceOn(limeLightName);
        }


        
    }


}

// {
//     "Results": {
//         "Bardcode": [],
//         "Classifier": [],
//         "Detector": [],
//         "Fiducial": [],
//         "Retro": [ we want this and to see which has the biggest ta
//             {
//                 "pts": [],
//                 "t6c_ts": [],
//                 "t6r_fs": [],
//                 "t6r_ts": [],
//                 "t6t_cs": [],
//                 "t6t_rs": [],
//                 "ta": 0.04886259883642197,
//                 "tx": 12.235940178638387,
//                 "tx_nocross": 13.889738258203792,
//                 "txp": 651.999267578125,
//                 "ty": -10.382303515705658,
//                 "ty_nocross": -10.94898055031723,
//                 "typ": 504.20947265625
//             },
//             {
//                 "pts": [],
//                 "t6c_ts": [],
//                 "t6r_fs": [],
//                 "t6r_ts": [],
//                 "t6t_cs": [],
//                 "t6t_rs": [],
//                 "ta": 0.0004670860944315791,
//                 "tx": -0.6196741780713211,
//                 "tx_nocross": 1.034123901494084,
//                 "txp": 471.2000427246094,
//                 "ty": -15.794492508498436,
//                 "ty_nocross": -16.36116954311001,
//                 "typ": 582.9000854492188
//             }
//         ],
//         "botpose": [
//             0.0,
//             0.0,
//             0.0,
//             0.0,
//             0.0,
//             0.0
//         ],
//         "botpose_avgarea": 0.0,
//         "botpose_avgdist": 0.0,
//         "botpose_span": 0.0,
//         "botpose_tagcount": 0,
//         "botpose_wpiblue": [
//             0.0,
//             0.0,
//             0.0,
//             0.0,
//             0.0,
//             0.0
//         ],
//         "botpose_wpired": [
//             0.0,
//             0.0,
//             0.0,
//             0.0,
//             0.0,
//             0.0
//         ],
//         "cl": 37.400001525878906,
//         "pID": 0.0,
//         "t6c_rs": [
//             0.0,
//             0.0,
//             0.0,
//             0.0,
//             0.0,
//             -0.0
//         ],
//         "tl": 13.421425819396973,
//         "ts": 3698157.3562580002,
//         "v": 1
//     }
// }
