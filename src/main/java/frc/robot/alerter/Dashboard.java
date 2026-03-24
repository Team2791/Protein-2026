package frc.robot.alerter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {

    static final String SHOOT_OK = "Status/ShootOk";
    static final String SHOOT_TIMER = "Status/ShootTimer";
    static final String PHASE = "Status/Phase";
    static final String SHOOT_WARNING = "Status/ShootWarning";

    static final double LEN_AUTO = 20.00;
    static final double LEN_TRANSITION = 10.00;
    static final double LEN_PHASE = 25.00;
    static final double LEN_ENDGAME = 30.00;
    static final double LEN_MATCH =
        LEN_AUTO + LEN_TRANSITION + 4 * LEN_PHASE + LEN_ENDGAME;

    static Dashboard instance = new Dashboard();

    Alliance winner = null;
    Alliance us = null;
    boolean warning = false;

    public static Dashboard getInstance() {
        return instance;
    }

    boolean fetchFMS() {
        if (DriverStation.isDSAttached() && us == null) {
            us = DriverStation.getAlliance().get();
        }

        if (DriverStation.isFMSAttached() && winner == null) {
            String msg = DriverStation.getGameSpecificMessage();
            if (msg.length() < 1) msg = "\0";

            winner = switch (msg.charAt(0)) {
                case 'R' -> Alliance.Red;
                case 'B' -> Alliance.Blue;
                default -> null;
            };
        }

        return us != null && winner != null;
    }

    void warn() {
        SmartDashboard.putBoolean(SHOOT_WARNING, warning);
        warning = !warning;
    }

    public void update() {
        double timedown = DriverStation.getMatchTime();
        double timer = LEN_MATCH - timedown;

        if (timer < LEN_AUTO) return;
        if (!fetchFMS()) return;

        System.out.println("time spent in match" + timer);
        double timerN = timer - LEN_AUTO - LEN_TRANSITION; // normalized for our calculations

        if (timerN < 0) {
            SmartDashboard.putNumber(SHOOT_TIMER, timerN);
            SmartDashboard.putNumber(PHASE, 0);
            SmartDashboard.putBoolean(SHOOT_OK, true);
            return;
        }

        int phaseZ = (int) MathUtil.clamp(timerN / LEN_PHASE, 0, 3); // zero based
        int phaseH = phaseZ + 1; // human readable
        double phaseT = LEN_PHASE - (timerN - phaseZ * LEN_PHASE); // time left in phase
        boolean even = phaseZ % 2 == 1; // "phase 1" == 0 in code. "winner" goes in even phases
        boolean won = us == winner; // we won

        SmartDashboard.putNumber(SHOOT_TIMER, phaseT);
        SmartDashboard.putNumber(PHASE, phaseH);
        SmartDashboard.putBoolean(SHOOT_OK, even == won);
        if (phaseT < 5 && phaseT > 0) warn();
        else SmartDashboard.putBoolean(SHOOT_WARNING, false);
    }
}
