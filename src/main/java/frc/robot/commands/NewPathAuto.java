package frc.robot.commands;

import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;

public class NewPathAuto extends ChoreoAuto {
    public NewPathAuto(SwerveDrive swerve, PigeonWrapper pigeon) {
        super("NewPath", swerve, pigeon);
    }
}
