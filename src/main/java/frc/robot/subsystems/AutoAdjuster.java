package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class AutoAdjuster extends SubsystemBase {
    private final String limeLightName = "limelight";

    public AutoAdjuster(){
        
    }
    @Override
    public void periodic(){
        boolean hasTarget = LimelightHelpers.getTV(limeLightName);
        if(hasTarget){
            double xOffset = LimelightHelpers.getTX(limeLightName);
            double yOffset = LimelightHelpers.getTY(limeLightName);
            double area = LimelightHelpers.getTA(limeLightName);
            SmartDashboard.putBoolean("Has Target", hasTarget);
            SmartDashboard.putNumber("X Offset", xOffset);
            SmartDashboard.putNumber("Y Offset", yOffset);
            SmartDashboard.putNumber("Area", area);
        } else {
            SmartDashboard.putBoolean("Has Target", false);
        }
        
    }
}
