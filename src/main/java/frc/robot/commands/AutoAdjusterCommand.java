package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Retro;

public class AutoAdjusterCommand extends Command{
    
    private final SwerveDrive swerve;
    private final double desiredArea; 

    public AutoAdjusterCommand(SwerveDrive swerve, double desiredArea){
        this.swerve = swerve;
        this.desiredArea = desiredArea;
        addRequirements(swerve);
    }


 

      public static LimelightTarget_Retro getLargestRetroTarget(LimelightTarget_Retro[] targets) {
           LimelightTarget_Retro largest = null;
            double maxArea = 0.0; 
            for (LimelightTarget_Retro target : targets) {
                if (target.ta > maxArea) {
                  maxArea = target.ta;
                    largest = target;
                }
            }

            if (largest != null){ SmartDashboard.putString("Largest Target Area", largest.toString());
            } else {
                SmartDashboard.putString("Largest Target Area", "None");
            }
            return largest;
        }

     public void adjustSwerveDrive(LimelightTarget_Retro[] targets){
        
        LimelightTarget_Retro largest = getLargestRetroTarget(targets);
        if (largest != null) {
        double rotation = Constants.PIDConstants.Drive.kSteerAngleP * largest.tx;
        double forwardSpeed = Constants.PIDConstants.Drive.kDriveVelocityP * (desiredArea - largest.ta);
        SmartDashboard.putNumber("Rotation", rotation);
        Translation2d trans = new Translation2d(
			forwardSpeed,0);
        swerve.drive(trans, rotation, true, false);
        }
    }

    @Override
	public void execute() {
        LimelightTarget_Retro[] targets = getRetroTargets();
        if (targets == null) return;
        SmartDashboard.putString("Retro Targets", targets.toString());
        adjustSwerveDrive(targets);
	}

    private LimelightTarget_Retro[] getRetroTargets(){
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(Constants.Vision.kLimelightName);

        return results.targets_Retro;

    }
}
