package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * Turela cu vedere (Limelight) + fallback pe Pinpoint.
 *
 * Fata de versiunea veche:
 *  1. Corectie de paralaxa: camera nu sta pe axa de rotatie a turelei, deci tx nu e
 *     unghiul pe care trebuie sa-l faca turela. Tag-ul e transformat din sistemul
 *     camerei in sistemul turelei (pivot) folosind distanta, iar unghiul se calculeaza
 *     fata de pivot.
 *  2. Compensare de latenta: tx e masurat pe un cadru vechi de ~20-60 ms. Daca turela
 *     s-a miscat intre timp, "currentAngle - tx" greseste. Se foloseste unghiul turelei
 *     din momentul capturii cadrului (istoric de unghiuri).
 *  3. getLatestResult() e citit o singura data pe bucla (inainte era citit de doua ori,
 *     putand amesteca doua cadre diferite).
 *  4. Fallback-ul pe Pinpoint roteste offsetul pivotului cu heading-ul robotului
 *     (inainte era adunat in coordonatele terenului, corect doar la heading 0).
 *
 * Conventie unghiuri: pozitiv = spre STANGA (CCW), la fel ca in codul vechi
 * (target = current - tx, iar tx pozitiv = tag la dreapta).
 */
public class TurretProfiledPIDControl2 implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private final LimeLight limeLight;
    private GamepadEx ct2;
    private ButtonReader btnManual;
    private boolean isManualMode = false;

    private final double TICKS_PER_DEGREE = 5.25;
    private final int TICKS_AT_CENTER = 455;
    private final int MIN_TICKS = 0;
    private final int MAX_TICKS = 910;

    // ===================== DE MASURAT PE ROBOT (inch) =====================
    // Pozitia camerei fata de AXA DE ROTATIE a turelei, in sistemul turelei:
    //   CAM_FWD_IN  = cat e camera in fata pivotului, pe directia de tragere (+ = in fata)
    //   CAM_LEFT_IN = cat e camera in lateral (+ = la stanga, privind in directia tragerii)
    //   CAM_YAW_DEG = cat e rotita camera fata de directia tragerii (+ = rotita spre stanga)
    public static double CAM_FWD_IN = 0.0;
    public static double CAM_LEFT_IN = 0.0;
    public static double CAM_YAW_DEG = 0.0;

    // Pozitia axei turelei fata de centrul de tracking al Pinpoint, in sistemul ROBOTULUI:
    //   + forward = spre fata robotului, + left = spre stanga robotului
    public static double PIVOT_FWD_IN = 0.0;
    public static double PIVOT_LEFT_IN = 0.0;
    // Unde priveste turela la TICKS_AT_CENTER fata de fata robotului (0 = inainte, 180 = inapoi)
    public static double TURRET_ZERO_HEADING_DEG = 0.0;

    // Punctul de ochire pe teren (inch), sistem de coordonate al Pinpoint
    public static double GOAL_X_BLUE = 0.0;
    public static double GOAL_X_RED = 144.0;
    public static double GOAL_Y = 144.0;

    // pythonOut[2] e in metri? (true -> se converteste in inch)
    public static boolean DISTANCE_IS_METERS = true;
    // =====================================================================

    public static double Kp = 0.042;
    public static double Ki = 0.00;
    public static double Kd = 0.005;
    public static double maxVelocity = 300.0;
    public static double maxAcceleration = 400.0;

    private ProfiledPIDController controller;
    private double maxTurretSpeed = 0.8;
    private double avgDistance = 0;
    private double persistentTargetAngle = 0;

    private final ElapsedTime lastTargetTimer = new ElapsedTime();
    private boolean isTrackingTag = true;
    private boolean hasTrackingLock = false;

    private final double VISION_TIMEOUT_SEC = .35;
    private final double ANGLE_TOLERANCE_DEG = .9;
    private final double MAX_ANGLE_RANGE = 90.0;

    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private boolean usePinpointFallback = true;

    // ---- istoric unghi turela pentru compensarea latentei ----
    private static final int HISTORY_SIZE = 64;
    private final long[] histTimeNs = new long[HISTORY_SIZE];
    private final double[] histAngle = new double[HISTORY_SIZE];
    private int histIndex = 0;
    private int histCount = 0;

    // ultimele valori calculate (utile in telemetrie)
    private double lastVisionErrorDeg = 0;
    private double lastLatencyMs = 0;

    public TurretProfiledPIDControl2(LimeLight limeLight, GamepadEx ct2) {
        this.limeLight = limeLight;
        this.ct2 = ct2;
        if (ct2 != null) {
            this.btnManual = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
        }
        setupController();
    }

    private void setupController() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        controller = new ProfiledPIDController(Kp, Ki, Kd, constraints);
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
        lastTargetTimer.reset();
        histCount = 0;
        recordAngle(currentAngle);
    }

    public void Update() {
        if (ct2 != null) {
            btnManual.readValue();
            if (btnManual.wasJustPressed()) {
                isManualMode = !isManualMode;
                if (!isManualMode) {
                    // ATENTIE: reset-ul pune 0 tickuri = MIN_TICKS (~-86.7 grade).
                    // E corect doar daca turela e impinsa la capatul MIN cand apesi butonul.
                    MotorTurela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    MotorTurela.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    persistentTargetAngle = 0;
                    controller.reset(ticksToAngle(0));
                    histCount = 0;
                }
            }
        }

        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = ticksToAngle(currentTicks);
        recordAngle(currentAngle);

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

        double currentDeadzone = 0.5;
        double currentTolerance = ANGLE_TOLERANCE_DEG;

        if (avgDistance > 0) {
            if (avgDistance < 90.55) {          // < 2.3 m
                currentDeadzone = 1.8;
                currentTolerance = 2.0;
            } else if (avgDistance < 130.0) {   // < 3.3 m
                currentDeadzone = 2.0;
                currentTolerance = 2.2;
            }
        }

        controller.setPID(Kp, Ki, Kd);
        controller.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        controller.setTolerance(currentTolerance);

        if (isTrackingTag) {
            boolean gotTarget = false;
            LLResult result = limeLight.getLimelight().getLatestResult(); // o singura citire
            if (result != null) {
                double[] pythonOut = result.getPythonOutput();
                if (pythonOut != null && pythonOut.length >= 3 && pythonOut[0] > 0.5) {
                    gotTarget = true;
                    lastTargetTimer.reset();
                    hasTrackingLock = true;

                    double txDeg = result.getTx();
                    double distIn = DISTANCE_IS_METERS ? pythonOut[2] * 39.3701 : pythonOut[2];

                    // unghiul turelei in momentul in care a fost facuta poza
                    lastLatencyMs = result.getCaptureLatency() + result.getTargetingLatency()
                            + result.getStaleness();
                    double angleAtCapture = angleAt(System.nanoTime() - (long) (lastLatencyMs * 1e6));

                    // tag-ul in sistemul pivotului turelei
                    double[] t = cameraToTurret(txDeg, distIn);
                    double errorLeftDeg = t[0];   // cat trebuie sa se roteasca turela spre stanga
                    double pivotDistIn = t[1];    // distanta pivot -> tag
                    lastVisionErrorDeg = errorLeftDeg;

                    // eroarea fata de pozitia ACTUALA, pentru deadzone
                    double newTarget = angleAtCapture + errorLeftDeg;
                    if (Math.abs(newTarget - currentAngle) > currentDeadzone
                            || Math.abs(newTarget - persistentTargetAngle) > currentDeadzone) {
                        persistentTargetAngle = newTarget;
                    }

                    if (avgDistance == 0)
                        avgDistance = pivotDistIn;
                    avgDistance = (0.75 * avgDistance) + (0.25 * pivotDistIn);
                }
            }

            if (!gotTarget && hasTrackingLock && usePinpointFallback
                    && lastTargetTimer.seconds() >= VISION_TIMEOUT_SEC) {
                persistentTargetAngle = getClosestAngleByPinpoint();
            }
        } else if (usePinpointFallback) {
            persistentTargetAngle = getClosestAngleByPinpoint();
        }

        persistentTargetAngle = Range.clip(persistentTargetAngle, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
        targetAngle = persistentTargetAngle;

        double power = controller.calculate(currentAngle, targetAngle);
        power = Range.clip(power, -maxTurretSpeed, maxTurretSpeed);

        if (currentTicks >= MAX_TICKS && power > 0)
            power = 0;
        if (currentTicks <= MIN_TICKS && power < 0)
            power = 0;

        MotorTurela.setPower(power);
    }

    /**
     * Transforma tag-ul din sistemul camerei in sistemul pivotului turelei.
     * Sistem: x = inainte (directia tragerii), y = stanga.
     * Returneaza {unghi spre tag fata de directia tragerii (grade, + = stanga), distanta pivot->tag}.
     */
    private double[] cameraToTurret(double txDeg, double distIn) {
        double tx = Math.toRadians(txDeg);
        // tx pozitiv = tag la dreapta => y negativ
        double xc = distIn * Math.cos(tx);
        double yc = -distIn * Math.sin(tx);

        double a = Math.toRadians(CAM_YAW_DEG);
        double xt = CAM_FWD_IN + xc * Math.cos(a) - yc * Math.sin(a);
        double yt = CAM_LEFT_IN + xc * Math.sin(a) + yc * Math.cos(a);

        return new double[] { Math.toDegrees(Math.atan2(yt, xt)), Math.hypot(xt, yt) };
    }

    private double getClosestAngleByPinpoint() {
        double headingRad = limeLight.GetRobotHeading();
        double cos = Math.cos(headingRad), sin = Math.sin(headingRad);

        // pozitia pivotului pe teren = pozitia robotului + offset rotit cu heading-ul
        double pivotX = limeLight.GetRobotX() + PIVOT_FWD_IN * cos - PIVOT_LEFT_IN * sin;
        double pivotY = limeLight.GetRobotY() + PIVOT_FWD_IN * sin + PIVOT_LEFT_IN * cos;

        double goalX = limeLight.IsBlue() ? GOAL_X_BLUE : GOAL_X_RED;
        double fieldAngle = Math.toDegrees(Math.atan2(GOAL_Y - pivotY, goalX - pivotX));

        return normalizeDeg(fieldAngle - Math.toDegrees(headingRad) - TURRET_ZERO_HEADING_DEG);
    }

    // ---------------- utilitare ----------------

    private double ticksToAngle(int ticks) {
        return (ticks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
    }

    private static double normalizeDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private void recordAngle(double angle) {
        histTimeNs[histIndex] = System.nanoTime();
        histAngle[histIndex] = angle;
        histIndex = (histIndex + 1) % HISTORY_SIZE;
        if (histCount < HISTORY_SIZE) histCount++;
    }

    /** Unghiul turelei la momentul tNs (interpolat din istoric). */
    private double angleAt(long tNs) {
        if (histCount == 0) return currentAngle;
        int newest = (histIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        if (tNs >= histTimeNs[newest]) return histAngle[newest];

        for (int i = 0; i < histCount - 1; i++) {
            int newer = (newest - i + HISTORY_SIZE) % HISTORY_SIZE;
            int older = (newer - 1 + HISTORY_SIZE) % HISTORY_SIZE;
            if (tNs >= histTimeNs[older]) {
                double span = histTimeNs[newer] - histTimeNs[older];
                if (span <= 0) return histAngle[older];
                double k = (tNs - histTimeNs[older]) / span;
                return histAngle[older] + k * (histAngle[newer] - histAngle[older]);
            }
        }
        int oldest = (newest - histCount + 1 + HISTORY_SIZE) % HISTORY_SIZE;
        return histAngle[oldest];
    }

    // ---------------- API public (neschimbat) ----------------

    public boolean isOnTarget() { return controller.atGoal(); }

    public void setTrackingTag(boolean track) { isTrackingTag = track; }

    public boolean isTrackingTag() { return isTrackingTag; }

    public double getCurrentDistance() { return avgDistance; }

    public void setUsePinpointFallback(boolean use) { this.usePinpointFallback = use; }

    public void setTargetAngle(double angleDegrees) {
        persistentTargetAngle = Range.clip(angleDegrees, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
    }

    public void setMaxSpeed(double speed) { maxTurretSpeed = Range.clip(speed, 0.0, 1.0); }

    public double getCurrentAngle() { return currentAngle; }

    public int getCurrentTicks() { return MotorTurela.getCurrentPosition(); }

    public double getTargetAngle() { return targetAngle; }

    public double getLastVisionErrorDeg() { return lastVisionErrorDeg; }

    public double getLastLatencyMs() { return lastLatencyMs; }

    public void resetTrackingLock() {
        hasTrackingLock = false;
        lastTargetTimer.reset();
    }

    public void Run() { Update(); }
}
