package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * Varianta 1: turela + shooter calculate cu FIZICA (aruncare oblica), dupa ideea din video-ul
 * "Shoot from Anywhere with Velocity Compensation" (Decode FTC).
 * Clasa completa, independenta (aceleasi valori PID / tickuri / deadzone ca TurretPinpointPIDControl).
 *
 * In loc de tabelul ShooterDistance se calculeaza direct:
 *  1. unghiul hood-ului si viteza mingii ca mingea sa treaca prin "punctul de trecere"
 *     (PASS_THROUGH_RADIUS_IN inaintea goal-ului, la SCORE_HEIGHT_IN deasupra iesirii mingii),
 *     cu unghiul de intrare SCORE_ANGLE_DEG;
 *  2. compensarea de miscare: viteza turelei se imparte in componenta spre goal si cea laterala;
 *     cea spre goal se scade din viteza orizontala necesara, cea laterala se anuleaza ochind
 *     putin in lateral -> hood, viteza si unghiul turelei recalculate;
 *  3. viteza mingii -> viteza flywheel (tick/s) si unghi hood -> pozitie servo, prin doua
 *     functii liniare care TREBUIE CALIBRATE pe robotul nostru.
 *
 * In TeleOp: shooter.SetTurretShot(turela).
 * Conventie: unghi turela + = stanga (CCW). 0 = TICKS_AT_CENTER. Unitati: inch, secunde.
 * Constantele public static se pot schimba LIVE din Panels (Configurables).
 */
@Configurable
public class TurretBallisticShotPID implements TurretShot {
    private DcMotorEx MotorTurela = null;
    private final RobotPinpoint robotPinpoint;
    private final GamepadEx ct2;
    private ButtonReader btnManual;
    private boolean isManualMode = false;

    // ---------- la fel ca in codul vechi ----------
    // Configurabile din Panels ca sa poata fi calibrate (valorile sunt cele din codul vechi)
    // Masurat cu "Calibrare Turela": 918 tickuri intre opritoare, plaja = 180° (+-90)
    // -> 918 / 180 = 5.1 tickuri/grad, centru = 918 / 2 = 459. (Inainte: 5.25 si 455.)
    public static double TICKS_PER_DEGREE = 5.1;
    public static int TICKS_AT_CENTER = 459;
    private final int MIN_TICKS = 0;
    private final int MAX_TICKS = 910;
    private final double MAX_ANGLE_RANGE = 90.0;

    public static double Kp = 0.032;
    public static double Ki = 0.00;
    public static double Kd = 0.005;
    public static double maxVelocity = 300.0;
    public static double maxAcceleration = 200.0;

    private double maxTurretSpeed = 0.6;

    // Deadzone / toleranta (grade) si puterea PID-ului, dupa distanta pana la goal (inch):
    //  - APROAPE: deadzone / toleranta mai mari, PID mai puternic (tinta se muta repede cand mergi)
    //  - DEPARTE: deadzone / toleranta mai mici, PID mai bland (corectii mici si precise)
    // Intre CLOSE_DISTANCE_IN si FAR_DISTANCE_IN valorile trec lin de la una la alta.
    public static double CLOSE_DISTANCE_IN = 90.55;   // sub asta = aproape (2.3 m)
    public static double FAR_DISTANCE_IN = 130.0;     // peste asta = departe (3.3 m)
    public static double DEADZONE_CLOSE_DEG = 2.0;
    public static double TOLERANCE_CLOSE_DEG = 2.2;
    public static double DEADZONE_FAR_DEG = 0.5;
    public static double ANGLE_TOLERANCE_DEG = 0.9;   // toleranta departe
    public static double KP_SCALE_CLOSE = 1.3;        // aproape: Kp x 1.3
    public static double KP_SCALE_FAR = 0.8;          // departe: Kp x 0.8
    // ---------------------------------------------

    // Unde priveste turela la TICKS_AT_CENTER fata de fata robotului (0 = inainte, ca in codul vechi)
    public static double TURRET_ZERO_HEADING_DEG = 0.0;

    // Optional: frecare statica. 0 = exact ca in codul vechi.
    public static double kS = 0.0;

    // ===================== GEOMETRIE ARUNCARE (de masurat) =====================
    // Punctul de referinta al goal-ului = tinta din RobotPinpoint (TARGET_BLUE/RED).
    public static double SCORE_HEIGHT_IN = 26.0;        // inaltimea punctului de trecere FATA DE IESIREA MINGII
    public static double SCORE_ANGLE_DEG = -30.0;       // unghiul de intrare (negativ = mingea coboara)
    public static double PASS_THROUGH_RADIUS_IN = 5.0;  // cat inainte de punctul de referinta trece mingea
    public static double G_IN_S2 = 386.09;              // gravitatia, inch/s^2

    // ===================== HOOD (de calibrat) =====================
    // Unghiul de lansare (grade fata de orizontala) pe care il poate face hood-ul.
    public static double HOOD_MIN_DEG = 45.0;
    public static double HOOD_MAX_DEG = 65.0;
    // pozitie servo = HOOD_SERVO_PER_DEG * grade + HOOD_SERVO_OFFSET
    // ATENTIE: valorile sunt ale echipei din video, NU ale robotului nostru.
    public static double HOOD_SERVO_PER_DEG = 0.0226;
    public static double HOOD_SERVO_OFFSET = -0.7443;
    public static double HOOD_SERVO_MIN = 0.51;         // limitele folosite acum de tabel
    public static double HOOD_SERVO_MAX = 0.60;

    // ===================== FLYWHEEL (de calibrat) =====================
    // viteza motor (tick/s) = FLY_TICKS_PER_IN_S * viteza_mingii(inch/s) + FLY_OFFSET
    // ATENTIE: valorile sunt ale echipei din video (94.501/12 si -187.96).
    public static double FLY_TICKS_PER_IN_S = 7.875;
    public static double FLY_OFFSET = -187.96;
    public static double FLY_MIN = 1100.0;              // limitele folosite acum de tabel
    public static double FLY_MAX = 1600.0;

    // ===================== COMPENSARE MISCARE =====================
    public static boolean COMPENSARE_MISCARE = true;
    public static double MIN_SPEED_IN_S = 3.0;          // sub asta robotul e considerat oprit
    // ===============================================================

    private final ProfiledPIDController controller;

    private double persistentTargetAngle = 0;
    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private double neededAngle = 0;      // unghiul calculat din Pinpoint in bucla asta
    private double currentDeadzone = DEADZONE_FAR_DEG;
    private double currentTolerance = ANGLE_TOLERANCE_DEG;
    private double currentKpScale = 1.0;

    private boolean autoAim = true;

    // rezultate fizica (ultima solutie valida)
    private boolean hasSolution = false;
    private boolean compensating = false;
    private double distanceIn = 0;       // axa turelei -> punctul de referinta al goal-ului
    private double horizontalIn = 0;     // x din formula
    private double hoodDeg = 0;
    private double launchSpeedInS = 0;
    private double flywheelVelocity = 0;
    private double hoodServo = 0;
    private double aimAngleDeg = 0;
    private double aimOffsetDeg = 0;
    private final ShooterDistance table = new ShooterDistance(); // doar pentru sugestia de calibrare

    /** ct2 poate fi null (autonomie) - atunci nu exista mod manual. */
    public TurretBallisticShotPID(RobotPinpoint robotPinpoint, GamepadEx ct2) {
        this.robotPinpoint = robotPinpoint;
        this.ct2 = ct2;
        if (ct2 != null) {
            this.btnManual = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
        }
        controller = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        controller.setTolerance(ANGLE_TOLERANCE_DEG);
    }

    public void LinkComponents(HardwareMap hwMap) {
        MotorTurela = hwMap.get(DcMotorEx.class, "MotorTurela");
    }

    public void Initialize(HardwareMap hwMap) {
        Initialize(hwMap, false);
    }

    public void Initialize(HardwareMap hwMap, boolean resetEncoder) {
        LinkComponents(hwMap);

        // valorile gasite de "Turela - Auto Tuning PID" (daca exista) inlocuiesc Kp / Kd / kS din cod
        double[] saved = TurretPidStore.Load();
        TurretPidStore.loadedFromFile = saved != null;
        if (saved != null) {
            Kp = saved[0];
            Kd = saved[1];
            kS = saved[2];
        }

        if (resetEncoder) {
            MotorTurela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        MotorTurela.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        MotorTurela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MotorTurela.setDirection(DcMotorEx.Direction.REVERSE);

        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = ticksToAngle(currentTicks);
        persistentTargetAngle = currentAngle;

        controller.reset(currentAngle);
        targetAngle = currentAngle;
    }

    public void Update() {
        TurretPidStore.ReportLoop(); // tunerul testeaza cu aceeasi durata a buclei
        if (ct2 != null) {
            btnManual.readValue();
            if (btnManual.wasJustPressed()) {
                isManualMode = !isManualMode;
                if (!isManualMode) {
                    // ATENTIE: 0 tickuri = MIN_TICKS. Apasa doar cu turela impinsa la capatul MIN.
                    MotorTurela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    MotorTurela.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    persistentTargetAngle = 0;
                    controller.reset(ticksToAngle(0));
                }
            }
        }

        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = ticksToAngle(currentTicks);

        ComputeShot();

        if (isManualMode) {
            double stick = ct2.getLeftX();
            if (Math.abs(stick) < 0.05)
                stick = 0;
            double manualPower = -stick * 0.7;
            if (currentTicks >= MAX_TICKS && manualPower > 0)
                manualPower = 0;
            if (currentTicks <= MIN_TICKS && manualPower < 0)
                manualPower = 0;
            MotorTurela.setPower(manualPower);
            return;
        }

        // Deadzone, toleranta si puterea PID dupa distanta (trec lin intre aproape si departe)
        double distance = distanceIn;
        double f = 1.0; // 1 = departe, 0 = aproape
        if (distance > 0) {
            f = Range.clip((distance - CLOSE_DISTANCE_IN) / Math.max(FAR_DISTANCE_IN - CLOSE_DISTANCE_IN, 1.0), 0.0, 1.0);
        }
        currentDeadzone = DEADZONE_CLOSE_DEG + (DEADZONE_FAR_DEG - DEADZONE_CLOSE_DEG) * f;
        currentTolerance = TOLERANCE_CLOSE_DEG + (ANGLE_TOLERANCE_DEG - TOLERANCE_CLOSE_DEG) * f;
        currentKpScale = KP_SCALE_CLOSE + (KP_SCALE_FAR - KP_SCALE_CLOSE) * f;

        controller.setPID(Kp * currentKpScale, Ki, Kd);
        controller.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        controller.setTolerance(currentTolerance);

        if (autoAim) {
            neededAngle = aimAngleDeg;
            // Codul vechi: if (|tx| > deadzone) target = currentAngle - tx.
            // Aici: tx = currentAngle - neededAngle, deci target = neededAngle.
            if (Math.abs(currentAngle - neededAngle) > currentDeadzone) {
                persistentTargetAngle = neededAngle;
            }
        }

        persistentTargetAngle = Range.clip(persistentTargetAngle, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
        targetAngle = persistentTargetAngle;

        double power = controller.calculate(currentAngle, targetAngle);
        double error = targetAngle - currentAngle;
        if (Math.abs(error) > currentTolerance) {
            power += Math.signum(error) * kS;
        }
        power = Range.clip(power, -maxTurretSpeed, maxTurretSpeed);

        if (currentTicks >= MAX_TICKS && power > 0)
            power = 0;
        if (currentTicks <= MIN_TICKS && power < 0)
            power = 0;

        MotorTurela.setPower(power);
    }

    /** Calculeaza lovitura (hood, viteza minge, unghi turela) o data pe bucla. */
    private void ComputeShot() {
        RobotPinpoint rp = robotPinpoint;

        double px = rp.GetTurretX();
        double py = rp.GetTurretY();
        double dx = rp.GetTargetX() - px;
        double dy = rp.GetTargetY() - py;
        distanceIn = Math.hypot(dx, dy);
        double goalTheta = Math.atan2(dy, dx);

        double x = Math.max(distanceIn - PASS_THROUGH_RADIUS_IN, 1.0);
        double y = SCORE_HEIGHT_IN;
        double a = Math.toRadians(SCORE_ANGLE_DEG);
        double minHood = Math.toRadians(HOOD_MIN_DEG);
        double maxHood = Math.toRadians(HOOD_MAX_DEG);

        // 1. lovitura cu robotul oprit
        double hood = Clamp(Math.atan(2 * y / x - Math.tan(a)), minHood, maxHood);
        double speed = LaunchSpeed(x, y, hood);
        double offset = 0;

        // 2. compensare de miscare
        double vx = rp.GetTurretVelX();
        double vy = rp.GetTurretVelY();
        compensating = COMPENSARE_MISCARE && Math.hypot(vx, vy) >= MIN_SPEED_IN_S && !Double.isNaN(speed);
        if (compensating) {
            double ux = Math.cos(goalTheta), uy = Math.sin(goalTheta);
            double vToward = vx * ux + vy * uy;    // spre goal
            double vLeft = -vx * uy + vy * ux;     // lateral, + = spre stanga fata de directia goal-ului

            double vz = speed * Math.sin(hood);
            double time = x / (speed * Math.cos(hood));
            double ivr = x / time - vToward;       // viteza orizontala necesara fata de robot, spre goal
            double nvr = Math.hypot(ivr, vLeft);   // + anularea derivei laterale
            double ndr = nvr * time;

            double hood2 = Clamp(Math.atan2(vz, nvr), minHood, maxHood);
            double speed2 = LaunchSpeed(ndr, y, hood2);
            if (!Double.isNaN(speed2) && ivr > 0) {
                hood = hood2;
                speed = speed2;
                offset = Math.atan2(-vLeft, ivr);  // ochim invers fata de deriva laterala
                x = ndr;
            } else {
                compensating = false;
            }
        }

        hasSolution = !Double.isNaN(speed);
        if (hasSolution) {
            horizontalIn = x;
            hoodDeg = Math.toDegrees(hood);
            launchSpeedInS = speed;
            flywheelVelocity = Clamp(FLY_TICKS_PER_IN_S * speed + FLY_OFFSET, FLY_MIN, FLY_MAX);
            hoodServo = Clamp(HOOD_SERVO_PER_DEG * hoodDeg + HOOD_SERVO_OFFSET, HOOD_SERVO_MIN, HOOD_SERVO_MAX);
        }
        aimOffsetDeg = Math.toDegrees(offset);
        aimAngleDeg = normalizeDeg(Math.toDegrees(goalTheta + offset) - rp.GetHeading() - TURRET_ZERO_HEADING_DEG);
    }

    /** Viteza de lansare (inch/s) ca mingea sa ajunga la (x, y) cu unghiul hood. NaN daca nu se poate. */
    private static double LaunchSpeed(double x, double y, double hood) {
        double cos = Math.cos(hood);
        double denom = 2 * cos * cos * (x * Math.tan(hood) - y);
        if (denom <= 0) return Double.NaN;
        return Math.sqrt(G_IN_S2 * x * x / denom);
    }

    private static double Clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * Sugestie pentru FLY_TICKS_PER_IN_S / FLY_OFFSET din tabelul ShooterDistance deja reglat:
     * unghiul hood-ului se afla din pozitia servo (cu HOOD_SERVO_*), viteza mingii din fizica,
     * apoi regresie liniara tick/s ~ viteza mingii. Are sens DUPA calibrarea HOOD_SERVO_*.
     * Returneaza {panta, offset, numar_puncte} sau null.
     */
    public double[] SuggestFlywheelFromTable() {
        if (HOOD_SERVO_PER_DEG == 0) return null;
        List<ShooterDistance.ShotParameters> rows = table.GetTable();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int n = 0;
        for (ShooterDistance.ShotParameters r : rows) {
            double hoodRad = Math.toRadians((r.hoodAngle - HOOD_SERVO_OFFSET) / HOOD_SERVO_PER_DEG);
            double v = LaunchSpeed(r.distanceCm / 2.54 - PASS_THROUGH_RADIUS_IN, SCORE_HEIGHT_IN, hoodRad);
            if (Double.isNaN(v)) continue;
            sx += v; sy += r.rpm; sxx += v * v; sxy += v * r.rpm;
            n++;
        }
        if (n < 2) return null;
        double den = n * sxx - sx * sx;
        if (Math.abs(den) < 1e-9) return null;
        double slope = (n * sxy - sx * sy) / den;
        return new double[] { slope, (sy - slope * sx) / n, n };
    }

    private double ticksToAngle(int ticks) {
        return (ticks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
    }

    private static double normalizeDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    // ---------------- API public ----------------

    public boolean isOnTarget() { return controller.atGoal(); }

    /** true = ocheste automat tinta; false = sta la unghiul dat cu setTargetAngle(). */
    public void setAutoAim(boolean enabled) { autoAim = enabled; }

    public boolean isAutoAim() { return autoAim; }

    /** Unghi fix (grade, + = stanga). Opreste ochirea automata. */
    public void setTargetAngle(double angleDegrees) {
        persistentTargetAngle = Range.clip(angleDegrees, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
        autoAim = false;
    }

    public void setMaxSpeed(double speed) { maxTurretSpeed = Range.clip(speed, 0.0, 1.0); }

    /** Distanta axa turelei -> tinta, in inch (din RobotPinpoint). */
    public double getCurrentDistance() { return robotPinpoint.GetDistanceToGoalInches(); }

    /** true daca tinta e in afara celor +-90 de grade (turela sta la limita). */
    public boolean isTargetOutOfRange() { return autoAim && Math.abs(neededAngle) > MAX_ANGLE_RANGE; }

    public boolean isManualMode() { return isManualMode; }

    @Override
    public double getCurrentAngle() { return currentAngle; }

    public int getCurrentTicks() { return MotorTurela.getCurrentPosition(); }

    public double getTargetAngle() { return targetAngle; }

    @Override
    public void UpdateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== TURELA ===");
        telemetry.addData("  PID", "Kp %.4f  Kd %.4f  kS %.3f  (%s)  bucla %.0f ms", Kp, Kd, kS,
                TurretPidStore.loadedFromFile ? "din auto-tuning" : "din cod", TurretPidStore.measuredLoopMs);
        telemetry.addData("  Mod", isManualMode ? "MANUAL" : (autoAim ? "AUTO (TurretBallisticShotPID)" : "UNGHI FIX"));
        telemetry.addData("  Unghi", "%.1f°", currentAngle);
        telemetry.addData("  Tinta", "%.1f° (calculat %.1f°)", targetAngle, neededAngle);
        telemetry.addData("  Eroare (tinta-unghi)", "%.2f°", targetAngle - currentAngle);
        telemetry.addData("  Deadzone / toleranta / Kp", "%.1f° / %.1f° / x%.2f (Kp %.4f)", currentDeadzone, currentTolerance, currentKpScale, Kp * currentKpScale);
        telemetry.addData("  Ticks", currentTicks);
        if (isTargetOutOfRange()) {
            telemetry.addLine("  Tinta in afara de +-90°!");
        }
        telemetry.addLine("  --- Fizica (varianta 1) ---");
        telemetry.addData("  Solutie", hasSolution ? "DA" : "NU (se pastreaza ultima)");
        telemetry.addData("  Compensare", compensating ? "DA" : "nu");
        telemetry.addData("  Dist / x", "%.0f cm / %.0f cm", distanceIn * 2.54, horizontalIn * 2.54);
        telemetry.addData("  Hood", "%.1f° -> servo %.3f", hoodDeg, hoodServo);
        telemetry.addData("  Viteza minge", "%.0f in/s (%.2f m/s) -> %.0f tick/s",
                launchSpeedInS, launchSpeedInS * 0.0254, flywheelVelocity);
        telemetry.addData("  Offset miscare", "%.1f°", aimOffsetDeg);
        double[] s = SuggestFlywheelFromTable();
        if (s != null) {
            telemetry.addData("  Sugestie din tabel", "FLY_TICKS_PER_IN_S=%.3f FLY_OFFSET=%.1f (%d pct)",
                    s[0], s[1], (int) s[2]);
        }
    }

    // ---------------- TurretShot (pentru Shooter, FieldDrawing, TeleOp) ----------------

    @Override
    public double getAimAngle() { return neededAngle; }

    @Override
    public double getZeroHeadingDeg() { return TURRET_ZERO_HEADING_DEG; }

    @Override
    public double GetShotDistanceMeters() { return distanceIn * 0.0254; }

    @Override
    public boolean HasDirectShot() { return hasSolution; }

    @Override
    public double GetFlywheelVelocity() { return flywheelVelocity; }

    @Override
    public double GetHoodServoPosition() { return hoodServo; }

    public void Run() { Update(); }
}
