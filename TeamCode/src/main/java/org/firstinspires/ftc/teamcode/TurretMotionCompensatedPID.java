package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Varianta 2: turela cu compensare de miscare ("tinta virtuala") + tabelul ShooterDistance.
 * Clasa completa, independenta (aceleasi valori PID / tickuri / deadzone ca TurretPinpointPIDControl).
 *
 * Mingea pleaca din shooter cu viteza robotului adaugata, deci "deriva" cat timp e in aer.
 * Ochim o TINTA VIRTUALA mutata invers fata de miscare:
 *
 *     tinta_virtuala = tinta - viteza_turelei * timp_de_zbor
 *
 * Timpul de zbor depinde de distanta, iar distanta de tinta virtuala -> se calculeaza de
 * ITERATII ori. Unghiul spre tinta virtuala merge in turela, distanta pana la ea in Shooter
 * (tabelul ShooterDistance), deci si flywheel-ul / hood-ul se adapteaza la miscare.
 * Cu robotul oprit (sub MIN_SPEED_IN_S) se comporta exact ca TurretPinpointPIDControl.
 *
 * In TeleOp: shooter.SetTurretShot(turela).
 * Conventie: unghi turela + = stanga (CCW). 0 = TICKS_AT_CENTER.
 * Constantele public static se pot schimba LIVE din Panels (Configurables).
 */
@Configurable
public class TurretMotionCompensatedPID implements TurretShot {
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

    // Deadzone / toleranta (grade), in functie de distanta (inch) - valorile din codul vechi
    public static double ANGLE_TOLERANCE_DEG = 0.9;   // departe (>= 3.3 m)
    public static double DEADZONE_FAR_DEG = 0.5;      // departe (>= 3.3 m)
    public static double CLOSE_DISTANCE_IN = 90.55;   // < 2.3 m
    public static double DEADZONE_CLOSE_DEG = 1.8;
    public static double TOLERANCE_CLOSE_DEG = 2.0;
    public static double MID_DISTANCE_IN = 130.0;     // < 3.3 m
    public static double DEADZONE_MID_DEG = 2.0;
    public static double TOLERANCE_MID_DEG = 2.2;
    // ---------------------------------------------

    // Unde priveste turela la TICKS_AT_CENTER fata de fata robotului (0 = inainte, ca in codul vechi)
    public static double TURRET_ZERO_HEADING_DEG = 0.0;

    // Optional: frecare statica. 0 = exact ca in codul vechi.
    public static double kS = 0.0;

    // ===================== COMPENSARE MISCARE (live din Panels) =====================
    public static boolean COMPENSARE_ACTIVA = true;

    // Timpul de zbor al mingii (s), interpolat liniar intre doua distante (cm).
    // VALORI ESTIMATE -> de masurat cu slow-motion (cadre de la iesirea din shooter pana la goal).
    public static double TOF_NEAR_CM = 120.0;
    public static double TOF_NEAR_S = 0.35;
    public static double TOF_FAR_CM = 375.0;
    public static double TOF_FAR_S = 0.75;

    // Cat de mult compensam (1.0 = teoretic). Mingea ramane in urma miscarii -> creste; o ia inainte -> scade.
    public static double LEAD_GAIN = 1.0;

    // Sub viteza asta robotul e considerat oprit (evita tremuratul din zgomotul vitezei).
    public static double MIN_SPEED_IN_S = 3.0;

    // Anticipare: unde va fi robotul peste LATENCY_S secunde. 0 = fara.
    public static double LATENCY_S = 0.0;

    public static int ITERATII = 3;

    // Feedforward pentru urmarire in miscare: putere per (grad/s) cu care se schimba tinta.
    // 0 = doar PID. Daca turela ramane in urma cand robotul merge/se roteste, creste cate 0.0005.
    public static double kV_TRACK = 0.0;
    public static double TRACK_FILTER = 0.3;
    public static double TRACK_MIN_RATE_DEG_S = 5.0;
    // ================================================================================

    private final ProfiledPIDController controller;

    private double persistentTargetAngle = 0;
    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private double neededAngle = 0;      // unghiul calculat din Pinpoint in bucla asta
    private double currentDeadzone = DEADZONE_FAR_DEG;
    private double currentTolerance = ANGLE_TOLERANCE_DEG;

    private boolean autoAim = true;

    // compensare
    private double virtX = 0, virtY = 0;
    private double shotDistanceIn = 0;
    private double aimAngleDeg = 0;
    private double timeOfFlightS = 0;
    private double turretSpeedInS = 0;
    private boolean compensating = false;
    private final ElapsedTime rateTimer = new ElapsedTime();
    private double lastAim = Double.NaN;
    private double aimRateDegS = 0;

    /** ct2 poate fi null (autonomie) - atunci nu exista mod manual. */
    public TurretMotionCompensatedPID(RobotPinpoint robotPinpoint, GamepadEx ct2) {
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

        ComputeVirtualTarget();

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

        // Deadzone si toleranta dupa distanta (ca in codul vechi, dar distanta vine din Pinpoint)
        double distance = shotDistanceIn;
        currentDeadzone = DEADZONE_FAR_DEG;
        currentTolerance = ANGLE_TOLERANCE_DEG;
        if (distance > 0) {
            if (distance < CLOSE_DISTANCE_IN) {
                currentDeadzone = DEADZONE_CLOSE_DEG;
                currentTolerance = TOLERANCE_CLOSE_DEG;
            } else if (distance < MID_DISTANCE_IN) {
                currentDeadzone = DEADZONE_MID_DEG;
                currentTolerance = TOLERANCE_MID_DEG;
            }
        }

        controller.setPID(Kp, Ki, Kd);
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
        if (autoAim && Math.abs(aimRateDegS) >= TRACK_MIN_RATE_DEG_S) {
            power += kV_TRACK * aimRateDegS; // feedforward urmarire (0 = dezactivat)
        }
        power = Range.clip(power, -maxTurretSpeed, maxTurretSpeed);

        if (currentTicks >= MAX_TICKS && power > 0)
            power = 0;
        if (currentTicks <= MIN_TICKS && power < 0)
            power = 0;

        MotorTurela.setPower(power);
    }

    /** Calculeaza tinta virtuala, unghiul spre ea si distanta (o data pe bucla). */
    private void ComputeVirtualTarget() {
        RobotPinpoint rp = robotPinpoint;

        double vx = rp.GetTurretVelX();
        double vy = rp.GetTurretVelY();
        turretSpeedInS = Math.hypot(vx, vy);
        compensating = COMPENSARE_ACTIVA && turretSpeedInS >= MIN_SPEED_IN_S;

        double px = rp.GetTurretX();
        double py = rp.GetTurretY();
        double headingDeg = rp.GetHeading();
        if (compensating && LATENCY_S > 0) {
            px += vx * LATENCY_S;
            py += vy * LATENCY_S;
            headingDeg += Math.toDegrees(rp.GetHeadingVelocityRad()) * LATENCY_S;
        }

        double gx = rp.GetTargetX();
        double gy = rp.GetTargetY();
        virtX = gx;
        virtY = gy;
        double dist = Math.hypot(gx - px, gy - py);
        timeOfFlightS = TimeOfFlight(dist);

        if (compensating) {
            for (int i = 0; i < ITERATII; i++) {
                virtX = gx - vx * timeOfFlightS * LEAD_GAIN;
                virtY = gy - vy * timeOfFlightS * LEAD_GAIN;
                dist = Math.hypot(virtX - px, virtY - py);
                timeOfFlightS = TimeOfFlight(dist);
            }
        }

        shotDistanceIn = dist;
        double fieldAngle = Math.toDegrees(Math.atan2(virtY - py, virtX - px));
        aimAngleDeg = normalizeDeg(fieldAngle - headingDeg - TURRET_ZERO_HEADING_DEG);

        double dt = rateTimer.seconds();
        rateTimer.reset();
        if (!Double.isNaN(lastAim) && dt > 1e-4) {
            double rate = normalizeDeg(aimAngleDeg - lastAim) / dt;
            aimRateDegS += TRACK_FILTER * (rate - aimRateDegS);
        }
        lastAim = aimAngleDeg;
    }

    /** Timp de zbor (s) din distanta (inch), interpolat liniar si limitat la capete. */
    private double TimeOfFlight(double distanceIn) {
        double cm = distanceIn * 2.54;
        if (TOF_FAR_CM <= TOF_NEAR_CM) return TOF_NEAR_S;
        double t = (cm - TOF_NEAR_CM) / (TOF_FAR_CM - TOF_NEAR_CM);
        t = Math.max(0.0, Math.min(1.0, t));
        return TOF_NEAR_S + t * (TOF_FAR_S - TOF_NEAR_S);
    }

    public double GetVirtualTargetX() { return virtX; }

    public double GetVirtualTargetY() { return virtY; }

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
        telemetry.addData("  Mod", isManualMode ? "MANUAL" : (autoAim ? "AUTO (TurretMotionCompensatedPID)" : "UNGHI FIX"));
        telemetry.addData("  Unghi", "%.1f°", currentAngle);
        telemetry.addData("  Tinta", "%.1f° (calculat %.1f°)", targetAngle, neededAngle);
        telemetry.addData("  Eroare (tinta-unghi)", "%.2f°", targetAngle - currentAngle);
        telemetry.addData("  Deadzone / toleranta", "%.1f° / %.1f°", currentDeadzone, currentTolerance);
        telemetry.addData("  Ticks", currentTicks);
        if (isTargetOutOfRange()) {
            telemetry.addLine("  Tinta in afara de +-90°!");
        }
        telemetry.addData("  Compensare", compensating ? "DA" : (COMPENSARE_ACTIVA ? "nu (robot oprit)" : "OPRITA"));
        telemetry.addData("  Viteza turela", "%.1f in/s", turretSpeedInS);
        telemetry.addData("  Timp zbor", "%.2f s", timeOfFlightS);
        telemetry.addData("  Tinta virtuala", "X %.1f  Y %.1f", virtX, virtY);
        telemetry.addData("  Dist. aruncare", "%.0f cm", shotDistanceIn * 2.54);
    }

    // ---------------- TurretShot (pentru Shooter, FieldDrawing, TeleOp) ----------------

    @Override
    public double getAimAngle() { return neededAngle; }

    @Override
    public double getZeroHeadingDeg() { return TURRET_ZERO_HEADING_DEG; }

    @Override
    public double GetShotDistanceMeters() { return shotDistanceIn * 0.0254; }

    @Override
    public boolean HasDirectShot() { return false; }

    @Override
    public double GetFlywheelVelocity() { return 0; }

    @Override
    public double GetHoodServoPosition() { return 0; }

    public void Run() { Update(); }
}
