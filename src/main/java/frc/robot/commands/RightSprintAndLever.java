package frc.robot.commands;

import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.PigeonWrapper;

public class RightSprintAndLever extends ChoreoAuto {
    public RightSprintAndLever(SwerveDrive swerve, PigeonWrapper pigeon) {
        super("RightSprintAndLever", swerve, pigeon);
    }
}
