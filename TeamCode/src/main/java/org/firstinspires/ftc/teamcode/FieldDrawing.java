package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Deseneaza pe terenul din Panels (ca Localization Test din Tuning), in timpul TeleOp-ului:
 *  - robotul (cerc albastru + linie de heading) si traseul lui (verde)
 *  - axa turelei (cerc mic) si punctul tinta (cerc)
 *  - linia GALBENA: unde AR TREBUI sa traga turela (axa turelei -> tinta)
 *  - linia ROSIE:   unde trage turela ACUM (din unghiul real al encoderului)
 * Daca turela e bine ochita, linia rosie se suprapune peste cea galbena.
 *
 * Panels: conectat la WiFi-ul robotului -> http://192.168.43.1:8001 -> tab-ul Field.
 */
public class FieldDrawing {
    private static final double ROBOT_RADIUS = 9;
    private static final int HISTORY_SIZE = 150;
    private static final double DRAW_PERIOD_MS = 50; // ~20 desene/s, ca sa nu incetineasca bucla

    private final Style robotStyle = new Style("", "#3F51B5", 0.75);
    private final Style historyStyle = new Style("", "#4CAF50", 0.5);
    private final Style targetStyle = new Style("", "#FFC107", 0.75);  // galben: unde ar trebui
    private final Style turretStyle = new Style("", "#F44336", 0.75);  // rosu: unde trage acum

    private FieldManager field;
    private boolean enabled = true;
    private final ElapsedTime drawTimer = new ElapsedTime();

    private final double[] histX = new double[HISTORY_SIZE];
    private final double[] histY = new double[HISTORY_SIZE];
    private int histIndex = 0;
    private int histCount = 0;

    public FieldDrawing() {
        try {
            field = PanelsField.INSTANCE.getField();
            field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        } catch (Exception e) {
            enabled = false; // fara Panels, TeleOp-ul merge normal
        }
    }

    public void Draw(RobotPinpoint rp, TurretPinpointPIDControl turret) {
        if (!enabled || drawTimer.milliseconds() < DRAW_PERIOD_MS)
            return;
        drawTimer.reset();

        try {
            double x = rp.GetX();
            double y = rp.GetY();
            double h = Math.toRadians(rp.GetHeading());

            // traseu
            histX[histIndex] = x;
            histY[histIndex] = y;
            histIndex = (histIndex + 1) % HISTORY_SIZE;
            if (histCount < HISTORY_SIZE) histCount++;

            field.setStyle(historyStyle);
            for (int i = 0; i < histCount - 1; i++) {
                int a = (histIndex - histCount + i + HISTORY_SIZE) % HISTORY_SIZE;
                int b = (a + 1) % HISTORY_SIZE;
                field.moveCursor(histX[a], histY[a]);
                field.line(histX[b], histY[b]);
            }

            // robot
            field.setStyle(robotStyle);
            field.moveCursor(x, y);
            field.circle(ROBOT_RADIUS);
            field.moveCursor(x + Math.cos(h) * ROBOT_RADIUS / 2, y + Math.sin(h) * ROBOT_RADIUS / 2);
            field.line(x + Math.cos(h) * ROBOT_RADIUS, y + Math.sin(h) * ROBOT_RADIUS);

            // axa turelei si tinta
            double tx = rp.GetTurretX();
            double ty = rp.GetTurretY();
            double gx = rp.GetTargetX();
            double gy = rp.GetTargetY();
            double dist = rp.GetDistanceToGoalInches();

            field.setStyle(targetStyle);
            field.moveCursor(gx, gy);
            field.circle(3);
            field.moveCursor(tx, ty);
            field.line(gx, gy);

            // unde priveste turela acum: heading robot + zero turela + unghi encoder
            double aim = h + Math.toRadians(TurretPinpointPIDControl.TURRET_ZERO_HEADING_DEG + turret.getCurrentAngle());
            field.setStyle(turretStyle);
            field.moveCursor(tx, ty);
            field.circle(1.5);
            field.moveCursor(tx, ty);
            field.line(tx + Math.cos(aim) * dist, ty + Math.sin(aim) * dist);

            field.update();
        } catch (Exception e) {
            enabled = false; // nu lasam desenul sa opreasca TeleOp-ul
        }
    }
}
