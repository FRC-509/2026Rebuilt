package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AutoAdjuster;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Retro;

public class AutoAdjusterCommand extends Command{
    
    private final AutoAdjuster autoAdjuster;

    public AutoAdjusterCommand(AutoAdjuster autoAdjuster){
        this.autoAdjuster = autoAdjuster;
    }

    @Override
	public void execute() {
        LimelightTarget_Retro[] targets = getRetroTargets();
        if (targets == null) return;
        SmartDashboard.putString("Retro Targets", targets.toString());
        autoAdjuster.adjustSwerveDrive(targets);
	}

    private LimelightTarget_Retro[] getRetroTargets(){
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(Constants.Vision.kLimelightName);

        return results.targets_Retro;

    }
}
