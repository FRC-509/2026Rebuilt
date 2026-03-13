package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GameManager extends SubsystemBase {
    public enum MatchPhase {
        DISABLED,
        AUTONOMOUS,
        TRANSITION_SHIFT,
        ALLIANCE_SHIFT_1,
        ALLIANCE_SHIFT_2,
        ALLIANCE_SHIFT_3,
        ALLIANCE_SHIFT_4,
        ENDGAME,
        TEST
    }

    private static final double TELEOP_START_SECONDS = 135.0;
    private static final double TRANSITION_SHIFT_START_SECONDS = 130.0;
    private static final double SHIFT_1_START_SECONDS = 105.0;
    private static final double SHIFT_2_START_SECONDS = 80.0;
    private static final double SHIFT_3_START_SECONDS = 55.0;
    private static final double SHIFT_4_START_SECONDS = 30.0;
    private static final double ENDGAME_START_SECONDS = 0.0;

    private MatchPhase currentPhase = MatchPhase.DISABLED;
    private double matchTimeSeconds = -1.0;
    private boolean autoWinConfirmed = false;
    private boolean hubActive = false;

    public void confirmAutoWin() {
        autoWinConfirmed = true;
    }

    public void clearAutoWin() {
        autoWinConfirmed = false;
    }

    public boolean isHubActive() {
        return hubActive;
    }

    public double getTimeToNextShift() {
        MatchPhase nextShift = getNextAllianceShiftPhase();
        if (nextShift == null) return -1.0;

        switch (nextShift) {
            case ALLIANCE_SHIFT_1:
                if (currentPhase == MatchPhase.AUTONOMOUS) {
                    return matchTimeSeconds + (TELEOP_START_SECONDS - TRANSITION_SHIFT_START_SECONDS);
                }
                return matchTimeSeconds - TRANSITION_SHIFT_START_SECONDS;
            case ALLIANCE_SHIFT_2:
                return matchTimeSeconds - SHIFT_1_START_SECONDS;
            case ALLIANCE_SHIFT_3:
                return matchTimeSeconds - SHIFT_2_START_SECONDS;
            case ALLIANCE_SHIFT_4:
                return matchTimeSeconds - SHIFT_3_START_SECONDS;
            default:
                return -1.0;
        }
    }

    public boolean isNextShiftActive() {
        MatchPhase nextShift = getNextAllianceShiftPhase();
        return nextShift != null && determineHubActive(nextShift);
    }

    private String getFormattedMatchTime() {
        if (matchTimeSeconds < 0.0) return "--:--";

        int totalSeconds = (int) Math.ceil(matchTimeSeconds);
        int minutes = totalSeconds / 60;
        int seconds = totalSeconds % 60;
        return String.format("%d:%02d", minutes, seconds);
    }

    private MatchPhase getNextAllianceShiftPhase() {
        switch (currentPhase) {
            case AUTONOMOUS:
            case TRANSITION_SHIFT:
                return MatchPhase.ALLIANCE_SHIFT_1;
            case ALLIANCE_SHIFT_1:
                return MatchPhase.ALLIANCE_SHIFT_2;
            case ALLIANCE_SHIFT_2:
                return MatchPhase.ALLIANCE_SHIFT_3;
            case ALLIANCE_SHIFT_3:
                return MatchPhase.ALLIANCE_SHIFT_4;
            default:
                return null;
        }
    }

    private MatchPhase determinePhase() {
        if (DriverStation.isTestEnabled()) return MatchPhase.TEST;
        if (DriverStation.isAutonomousEnabled()) return MatchPhase.AUTONOMOUS;
        if (DriverStation.isTeleopEnabled()) {
            if (matchTimeSeconds > TELEOP_START_SECONDS) return MatchPhase.DISABLED;
            if (matchTimeSeconds > TRANSITION_SHIFT_START_SECONDS) return MatchPhase.TRANSITION_SHIFT;
            if (matchTimeSeconds > SHIFT_1_START_SECONDS) return MatchPhase.ALLIANCE_SHIFT_1;
            if (matchTimeSeconds > SHIFT_2_START_SECONDS) return MatchPhase.ALLIANCE_SHIFT_2;
            if (matchTimeSeconds > SHIFT_3_START_SECONDS) return MatchPhase.ALLIANCE_SHIFT_3;
            if (matchTimeSeconds > SHIFT_4_START_SECONDS) return MatchPhase.ALLIANCE_SHIFT_4;
            if (matchTimeSeconds >= ENDGAME_START_SECONDS) return MatchPhase.ENDGAME;
        }
        return MatchPhase.DISABLED;
    }

    private boolean determineHubActive(MatchPhase phase) {
        switch (phase) {
            case AUTONOMOUS:
            case TRANSITION_SHIFT:
            case ENDGAME:
                return true;
            case ALLIANCE_SHIFT_1:
            case ALLIANCE_SHIFT_3:
                return !autoWinConfirmed;
            case ALLIANCE_SHIFT_2:
            case ALLIANCE_SHIFT_4:
                return autoWinConfirmed;
            default:
                return false;
        }
    }

    @Override
    public void periodic() {
        matchTimeSeconds = DriverStation.getMatchTime();
        MatchPhase updatedPhase = determinePhase();

        if (updatedPhase == MatchPhase.AUTONOMOUS && currentPhase != MatchPhase.AUTONOMOUS) {
            autoWinConfirmed = false;
        }

        currentPhase = updatedPhase;
        hubActive = determineHubActive(currentPhase);

        SmartDashboard.putString("Game/Phase", currentPhase.name());
        SmartDashboard.putString("Game/MatchTimeFormatted", getFormattedMatchTime());
        SmartDashboard.putBoolean("Game/WonAuto", autoWinConfirmed);
        SmartDashboard.putBoolean("Game/HubActive", hubActive);
        SmartDashboard.putNumber("Game/TimeToNextShiftSeconds", getTimeToNextShift());
    }
}
