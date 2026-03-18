package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class AutoAdjuster extends SubsystemBase {
    private final String limeLightName = "limelight-intake";
    private final NetworkTable limeLightTable;

    public AutoAdjuster(){
        limeLightTable = NetworkTableInstance.getDefault().getTable(limeLightName);
        
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
        }

        
    }
}
