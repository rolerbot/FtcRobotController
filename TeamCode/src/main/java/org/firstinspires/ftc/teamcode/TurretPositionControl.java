package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * Control de poziție pentru turelă - setează POZIȚIA țintă în loc de putere
 * Centrat pe AprilTag folosind encoder position control
 */
public class TurretPositionControl implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private LimeLight limeLight;
    private Shooter shooter;

    // Encoder settings - Reset at RIGHT barrier (CW Limit)
    private final double TICKS_PER_DEGREE = 5.25; // 435 RPM Motor Scale
    private final int TICKS_AT_CENTER = 472; // Center is 90 deg (472 ticks) from the Right
    private final int MIN_TICKS = 0; // RIGHT BARRIER (Physical Start)
    private final int MAX_TICKS = 915; // LEFT BARRIER (180 deg swing)

    // ✅ PID coefficients pentru control de poziție - CUSTOM OVERRIDE
    // Aceștia suprascriu PID-ul intern al SDK-ului pentru RUN_TO_POSITION
    private double kp = 5.0; // Proportional - Crește pentru răspuns mai rapid
    private double ki = 0.1; // Integral - Elimină eroarea steady-state
    private double kd = 0.5; // Derivative - Reduce oscilațiile
    private double kf = 10.0; // Feedforward - Viteză de bază (ticks/sec la max power) // 12

    // Flag pentru a activa/dezactiva PID custom
    private boolean useCustomPID = true;

    // ✅ VITEZĂ MAXIMĂ - Crește pentru mișcare mai rapidă (0.0 - 1.0)
    private double maxTurretSpeed = 1.0; // Putere maximă pentru RUN_TO_POSITION

    private double avgDistance = 0; // Filtered distance for smoothing
    private double persistentTargetAngle = 0; // Target angle in degrees
    private int targetTicks = TICKS_AT_CENTER; // Target position in ticks

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime lastTargetTimer = new ElapsedTime();

    private boolean isTrackingTag = true;
    private boolean hasTrackingLock = false;

    // ✅ TRACKING INERTIAL - Compensare rotație robot
    private double filteredRobotTurnSpeed = 0; // Viteza de rotație filtrată a robotului (deg/sec)
    private final double VISION_TIMEOUT_SEC = 0.7; // ⬆️ Crescut pentru shooting din mișcare (0.3→0.5s)
    private boolean useInertialTracking = true; // Flag pentru activare/dezactivare

    // Tolerance for "on target"
    private final double ANGLE_TOLERANCE_DEG = 1.0;
    private final int TICK_TOLERANCE = (int) (ANGLE_TOLERANCE_DEG * TICKS_PER_DEGREE);

    // Debug state variables
    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;

    public TurretPositionControl(LimeLight limeLight, Shooter shooter) {
        this.limeLight = limeLight;
        this.shooter = shooter;
    }

    public TurretPositionControl(LimeLight limeLight) {
        this.limeLight = limeLight;
        this.shooter = null;
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

        // ✅ CONTROL DE POZIȚIE - Setează modul RUN_TO_POSITION
        MotorTurela.setTargetPosition(MotorTurela.getCurrentPosition());
        MotorTurela.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MotorTurela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MotorTurela.setDirection(DcMotorEx.Direction.REVERSE);
        MotorTurela.setTargetPositionTolerance(TICK_TOLERANCE);
        // ✅ OVERRIDE PID INTERN - Aplică coeficienți custom
        if (useCustomPID) {
            MotorTurela.setPositionPIDFCoefficients(12);
        }

        // Setează puterea maximă pentru RUN_TO_POSITION
        MotorTurela.setPower(maxTurretSpeed);

        // Inițializează target-ul la poziția curentă
        currentTicks = MotorTurela.getCurrentPosition();
        targetTicks = currentTicks;
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
        persistentTargetAngle = currentAngle;

        // Inițializează timer pentru tracking inertial
        timer.reset();
        lastTargetTimer.reset();
        filteredRobotTurnSpeed = 0;
    }

    public void Update() {
        // 1. Obține poziția curentă
        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;

        // ✅ 1.5. Obține viteza de rotație a robotului pentru tracking inertial
        double rawRobotTurnVelocity = limeLight.GetRobotHeadingVelocity(); // deg/sec

        // Filtrează viteza de rotație pentru smooth tracking
        // ⬆️ Optimizat pentru mișcare: 60% nou + 40% vechi (mai responsiv decât 70/30)
        filteredRobotTurnSpeed = (0.9 * rawRobotTurnVelocity) + (0.1 * filteredRobotTurnSpeed);

        // 2. Determină unghiul țintă (HYBRID: Vision prioritized, Odometry fallback)
        if (isTrackingTag) {
            LLResult result = limeLight.getLimelight().getLatestResult();

            if (result != null && result.isValid()) {
                lastTargetTimer.reset();
                hasTrackingLock = true;

                // VISION UPDATE: Calculează unghiul țintă pentru centrare
                double tx = result.getTx(); // Offset orizontal în grade

                // ✅ COMPENSARE LATENȚĂ - Look-ahead bazat pe viteza robotului
                double totalLatencySeconds = (result.getCaptureLatency() + 11.0) / 1000.0;
                double latencyCorrection = 0;

                if (useInertialTracking) {
                    latencyCorrection = rawRobotTurnVelocity * totalLatencySeconds;
                }

                // Unghiul țintă = unghiul curent - (offset + compensare latență)
                persistentTargetAngle = currentAngle - (tx + latencyCorrection);

                // Update shooter distance from Tag
                double rawDistanceInches = limeLight.GetDistance2DToAprilTagFromRobotCenter(currentAngle);
                double POI_OFFSET_INCHES = 18.11;
                double distanceInches = rawDistanceInches + POI_OFFSET_INCHES;

                if (rawDistanceInches > 0) {
                    if (avgDistance == 0)
                        avgDistance = distanceInches;
                    avgDistance = (0.70 * avgDistance) + (0.30 * distanceInches);
                }

                // ✅ RELOCALIZARE PINPOINT: Actualizează poziția odometry folosind MT2 (cu
                // stability check)
                limeLight.RelocalizePinpointWithMT2(currentAngle);

            } else {
                // ✅ VISION LOST - Fallback logic
                if (hasTrackingLock && useInertialTracking && lastTargetTimer.seconds() < VISION_TIMEOUT_SEC) {
                    // 1. TRACKING INERTIAL (Compensare rotație rapidă)
                    double inertialDelta = -filteredRobotTurnSpeed * timer.seconds();
                    persistentTargetAngle += inertialDelta;
                } else {
                    // 2. ODOMETRY FALLBACK (Pinpoint calculated field angle)
                    persistentTargetAngle = getClosestAngleByPinpoint();
                }
            }
        } else {
            // Tracking dezactivat manual - Folosește odometria per-default
            persistentTargetAngle = getClosestAngleByPinpoint();
        }

        // Reset timer pentru tracking inertial
        timer.reset();

        // 3. Limitează unghiul țintă la ±90 grade față de centru
        final double MAX_ANGLE_RANGE = 90.0; // ±90 grade
        persistentTargetAngle = Range.clip(persistentTargetAngle, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);

        // 4. Convertește unghiul țintă în ticks
        targetTicks = (int) (persistentTargetAngle * TICKS_PER_DEGREE + TICKS_AT_CENTER);

        // Asigură-te că targetTicks este în limite
        targetTicks = Range.clip(targetTicks, MIN_TICKS, MAX_TICKS);

        // 5. ✅ SETEAZĂ POZIȚIA ȚINTĂ
        MotorTurela.setTargetPosition(targetTicks);

        // Motor-ul va merge automat la poziția țintă folosind PID-ul intern
        // Asigură-te că puterea este setată corect
        MotorTurela.setPower(maxTurretSpeed);

        // Update debug state
        targetAngle = persistentTargetAngle;
    }

    /**
     * Verifică dacă turela este aproape de poziția țintă
     */
    public boolean isOnTarget() {
        int error = Math.abs(targetTicks - currentTicks);
        return error <= TICK_TOLERANCE;
    }

    /**
     * Calculează unghiul dintre axa X (frontul robotului) și direcția către colțul
     * de scoring al field-ului (bazat pe alianță).
     *
     * Folosește datele de poziție de la Pinpoint (prin LimeLight).
     * Sistem de coordonate: Centrul (0,0) este în STÂNGA-JOS (colțul albastru)
     *
     * Ținta se schimbă automat în funcție de alianță:
     * - ALBASTRU: Colț stânga-jos (0, 0) - CENTRUL FIELD
     * - ROȘU: Colț dreapta-sus (144, 144) - OPUS
     *
     * @return Unghiul în grade (-180 la +180) relativ la frontul robotului
     *         Pozitiv = stânga, Negativ = dreapta
     */
    private double getClosestAngleByPinpoint() {
        // ✅ Determină ținta bazat pe alianță
        // SISTEM COORDONATE: (0,0) = stânga-jos (colț albastru)
        double TARGET_X;
        double TARGET_Y;

        if (limeLight.IsBlue()) {
            // ALIANȚA ALBASTRĂ - Vizează colțul stânga-jos (CENTRUL)
            TARGET_X = 144.0; // Stânga (origin)
            TARGET_Y = 0.0; // Jos (origin)
        } else {
            // ALIANȚA ROȘIE - Vizează colțul dreapta-sus (OPUS)
            TARGET_X = 144.0; // Dreapta (144 inches)
            TARGET_Y = 144.0; // Sus (144 inches)
        }

        // Obține poziția curentă a robotului de la Pinpoint (prin LimeLight)
        double robotX = limeLight.GetRobotX();
        double robotY = limeLight.GetRobotY();
        double robotHeading = limeLight.GetRobotHeading();

        // Calculează vectorul de la robot la țintă
        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;

        // Calculează unghiul absolut către țintă (în coordonate field)
        double angleToTargetRadians = Math.atan2(deltaY, deltaX);
        double angleToTargetDegrees = Math.toDegrees(angleToTargetRadians);

        // Convertește din coordonate field în unghi relativ la robot
        double relativeAngle = angleToTargetDegrees - robotHeading;

        // Normalizează unghiul la intervalul [-180, +180]
        while (relativeAngle > 180) {
            relativeAngle -= 360;
        }
        while (relativeAngle < -180) {
            relativeAngle += 360;
        }

        return relativeAngle;
    }

    /**
     * Obține eroarea curentă în grade
     */
    public double getErrorDegrees() {
        return targetAngle - currentAngle;
    }

    /**
     * Obține eroarea curentă în ticks
     */
    public int getErrorTicks() {
        return targetTicks - currentTicks;
    }

    public void setTrackingTag(boolean track) {
        isTrackingTag = track;
    }

    public boolean isTrackingTag() {
        return isTrackingTag;
    }

    public double getCurrentDistance() {
        return limeLight.GetDistance2DToAprilTagFromRobotCenter(currentAngle);
    }

    /**
     * Setează manual poziția țintă (în grade relative la centru)
     * Limitat la ±90 grade
     */
    public void setTargetAngle(double angleDegrees) {
        // Limitează la ±90 grade
        persistentTargetAngle = Range.clip(angleDegrees, -90.0, 90.0);
        targetTicks = (int) (persistentTargetAngle * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        MotorTurela.setTargetPosition(targetTicks);
    }

    /**
     * Setează manual poziția țintă (în ticks absolute)
     * Se asigură că rămâne în intervalul ±90 grade
     */
    public void setTargetTicks(int ticks) {
        targetTicks = ticks;
        persistentTargetAngle = (targetTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;

        // Verifică și limitează la ±90 grade
        if (persistentTargetAngle > 90.0) {
            persistentTargetAngle = 90.0;
            targetTicks = (int) (90.0 * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        } else if (persistentTargetAngle < -90.0) {
            persistentTargetAngle = -90.0;
            targetTicks = (int) (-90.0 * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        }

        MotorTurela.setTargetPosition(targetTicks);
    }

    /**
     * Setează coeficienții PID pentru control de poziție
     * Aplică override la PID-ul intern al SDK-ului
     */
    public void setPIDCoefficients(double p, double i, double d, double f) {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.kf = f;

        // ✅ Aplică efectiv la motor
        PIDFCoefficients customPID = new PIDFCoefficients(p, i, d, f);
        MotorTurela.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, customPID);

        useCustomPID = true;
    }

    /**
     * Obține coeficienții PID actuali de la motor
     */
    public PIDFCoefficients getPIDCoefficients() {
        return MotorTurela.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Activează/dezactivează PID-ul custom
     * Dacă dezactivat, folosește PID-ul default al SDK-ului
     */
    public void setUseCustomPID(boolean use) {
        useCustomPID = use;
        if (use) {
            setPIDCoefficients(kp, ki, kd, kf);
        } else {
            // Resetează la valorile default ale SDK-ului
            // Valorile default depind de motorul folosit
        }
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public int getCurrentTicks() {
        return currentTicks;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public double getKf() {
        return kf;
    }

    /**
     * Setează viteza maximă a turelei (0.0 - 1.0)
     * Valori mai mari = mișcare mai rapidă
     * Recomandat: 0.5 - 1.0 pentru tracking precis
     */
    public void setMaxSpeed(double speed) {
        maxTurretSpeed = Range.clip(speed, 0.0, 1.0);
        MotorTurela.setPower(maxTurretSpeed);
    }

    /**
     * Obține viteza maximă curentă
     */
    public double getMaxSpeed() {
        return maxTurretSpeed;
    }

    /**
     * Resetează tracking lock (folosit când începe TeleOp nou)
     */
    public void resetTrackingLock() {
        hasTrackingLock = false;
        filteredRobotTurnSpeed = 0;
        timer.reset();
        lastTargetTimer.reset();
    }

    /**
     * Activează/dezactivează tracking inertial
     * Când e activat, compensează rotația robotului chiar și când pierde tag-ul
     * temporar
     */
    public void setUseInertialTracking(boolean use) {
        useInertialTracking = use;
    }

    /**
     * Verifică dacă tracking-ul inertial este activ
     */
    public boolean isUsingInertialTracking() {
        return useInertialTracking;
    }

    /**
     * Obține viteza de rotație filtrată a robotului (deg/sec)
     */
    public double getFilteredRobotTurnSpeed() {
        return filteredRobotTurnSpeed;
    }

    /**
     * Verifică dacă turela are tracking lock (a văzut vreodată un tag)
     */
    public boolean hasTrackingLock() {
        return hasTrackingLock;
    }

    /**
     * Obține timpul de la ultima detecție validă a tag-ului (secunde)
     */
    public double getTimeSinceLastTag() {
        return lastTargetTimer.seconds();
    }

    /**
     * Verifică dacă turela urmărește activ un tag (nu e în timeout)
     */
    public boolean isActivelyTracking() {
        return hasTrackingLock && lastTargetTimer.seconds() < VISION_TIMEOUT_SEC;
    }

    public void Run() {
        Update();
    }
}
