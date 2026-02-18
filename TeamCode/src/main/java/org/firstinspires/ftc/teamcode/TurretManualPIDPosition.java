package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;

public class TurretManualPIDPosition implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private LimeLight limeLight;
    private Shooter shooter;

    // ✅ OPTIMIZAT pentru viteză mai mare
    private double kp = 0.023; // Crescut de la 0.023 → 0.035 pentru răspuns mai rapid
    private double kd = 0.0012; // Crescut de la 0.001 → 0.0015 pentru amortizare mai bună
    private double avgDistance = 0; // Filtered distance for smoothing
    private double kp_rotation = 0.85; // Crescut de la 0.8 → 1.2 pentru tracking agresiv în mișcare
    private double kd_rotation = 0.009; // Crescut de la 0.006 → 0.009
    private double kv_head = 0.035; // Crescut de la 0.02 → 0.035 pentru compensare mai puternică
    private double persistentTargetAngle = 0; // Persistent target for tracking through blur
    private final double MAX_SLEW = 1.0; // Unlimited for tuning response
    private ElapsedTime deltaTimer = new ElapsedTime(); // For inertial integration
    private double lastError = 0;
    private double filteredRobotTurnSpeed = 0; // Low-pass filter for noise
    private double smoothDTerm = 0; // Filtered derivative
    private final double angleTolerance = 1.0;
    private final double MAX_POWER = 1.0; // ALLOW FULL SPEED

    // Encoder settings - Reset at RIGHT barrier (CW Limit)
    private final double TICKS_PER_DEGREE = 5.25; // 435 RPM Motor Scale
    private final int TICKS_AT_CENTER = 472; // Center is 90 deg (472 ticks) from the Right
    private final int MIN_TICKS = 0; // RIGHT BARRIER (Physical Start)
    private final int MAX_TICKS = 915; // LEFT BARRIER (180 deg swing)
    private final int LIMIT_BUFFER = 50; // Ticks buffer for soft deceleration

    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean isTrackingTag = true;
    private final ElapsedTime lastTargetTimer = new ElapsedTime();
    private final double VISION_TIMEOUT_SEC = 0.5;

    // Debug state variables
    private double currentAngle = 0;
    private double targetAngle = 0;
    private double currentError = 0;

    private boolean hasTrackingLock = false; // Flag: have we seen a tag yet?

    public TurretManualPIDPosition(LimeLight limeLight, Shooter shooter) {
        this.limeLight = limeLight;
        this.shooter = shooter;
    }

    public TurretManualPIDPosition(LimeLight limeLight) {
        this.limeLight = limeLight;
        this.shooter = null;
    }

    public void LinkComponents(HardwareMap hwMap) {
        MotorTurela = hwMap.get(DcMotorEx.class, "MotorTurela");
    }

    public void Initialize(HardwareMap hwMap) {
        Initialize(hwMap, false); // Default: don't reset encoder (for TeleOp)
    }

    public void Initialize(HardwareMap hwMap, boolean resetEncoder) {
        LinkComponents(hwMap);

        if (resetEncoder) {
            // Only reset encoder in calibration mode - preserves position in TeleOp
            MotorTurela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        MotorTurela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        MotorTurela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MotorTurela.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void Update() {
        double deltaTime = timer.seconds();
        timer.reset();

        // 1. Get current turret state
        int currentTicks = MotorTurela.getCurrentPosition();
        // Convert to degrees where 0 is Forward, Left is Positive, Right is Negative
        double currentTurretAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;

        // 2. Compensate Pinpoint for turret rotation
        // DISABLED FOR NOW TO DEBUG JITTER
        // limeLight.CompensatePinpointForTurret(currentTurretAngle);

        // 3. Determine target angle
        // We use its previous value as the base (Persistent)
        deltaTime = deltaTimer.seconds();
        deltaTimer.reset();

        // ✅ TRACKING INERTIAL ACTIVAT - permite shooting din mișcare!
        // Obține viteza de rotație a robotului de la Pinpoint (prin LimeLight)
        double rawRobotTurnVelocity = limeLight.GetRobotHeadingVelocity(); // ACTIVAT pentru mișcare!

        if (isTrackingTag) {
            LLResult result = limeLight.getLimelight().getLatestResult();

            if (result != null && result.isValid()) {
                lastTargetTimer.reset();

                // VISION UPDATE: Aim at the tag + Look Ahead
                double tx = result.getTx();
                // User has High-Speed settings (90fps) -> Low Latency (~11-15ms)
                double totalLatencySeconds = (result.getCaptureLatency() + 11.0) / 1000.0;
                double latencyCorrection = rawRobotTurnVelocity * totalLatencySeconds;

                persistentTargetAngle = currentTurretAngle - (tx + latencyCorrection);

                // Update shooter distance from Tag
                double rawDistanceInches = limeLight.GetDistance2DToAprilTagFromRobotCenter(currentTurretAngle);

                // POI OFFSET: Target is 0.46m (18.11 inches) BEHIND the tag.
                // We add this to the measured distance.
                double POI_OFFSET_INCHES = 18.11;
                double distanceInches = rawDistanceInches + POI_OFFSET_INCHES;

                if (rawDistanceInches > 0) {
                    // Initialize if first reading
                    if (avgDistance == 0)
                        avgDistance = distanceInches;

                    // Strong Low-Pass Filter (Smooths jitter strongly)
                    // 90% Old Value, 10% New Value
                    avgDistance = (0.90 * avgDistance) + (0.10 * distanceInches);

                    if (shooter != null) {
                        //shooter.SetCustomDistanceMeters(avgDistance * 0.0254);
                    }
                }
                // We have seen a tag! Enable tracking lock so we don't snap back to 0 randomly.
                hasTrackingLock = true;

            } else {
                // ✅ TRACKING INERTIAL ACTIV - compensează când pierde tag-ul!
                // Logic Flow:
                // 1. Dacă a văzut tag-ul înainte ȘI nu a depășit timeout-ul → compensează rotația
                // 2. Dacă a depășit timeout-ul → menține poziția curentă
                // 3. Dacă nu a văzut niciodată tag → revine la centru

                if (hasTrackingLock && lastTargetTimer.seconds() < VISION_TIMEOUT_SEC) {
                    // Compensare activă - estimează unde s-a mutat tag-ul
                    double inertialDelta = -rawRobotTurnVelocity * deltaTime;
                    persistentTargetAngle += inertialDelta;
                } else if (hasTrackingLock) {
                    // Timeout depășit - menține ultima poziție cunoscută
                    persistentTargetAngle = currentTurretAngle;
                } else {
                    // Nu a văzut niciodată tag - revine la centru
                    persistentTargetAngle = 0;
                }
            }
        } else {
            // Tracking disabled
            persistentTargetAngle = currentTurretAngle;
        }

        double targetRelativeAngle = persistentTargetAngle;

        // Clamp target angle to physical turret limits
        double minAngle = (MIN_TICKS - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
        double maxAngle = (MAX_TICKS - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
        targetRelativeAngle = Range.clip(targetRelativeAngle, minAngle, maxAngle);

        // 4. PD Control
        double error = targetRelativeAngle - currentTurretAngle;

        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;

        // Use Dynamic PID Scaling based on filtered robot rotation velocity
        // 1. LOW-PASS FILTER: (Alpha = 0.8 for faster response)
        // ✅ Viteza de rotație este DEJA obținută mai sus - NU o mai reseta la 0!
        filteredRobotTurnSpeed = (0.8 * rawRobotTurnVelocity) + (0.2 * filteredRobotTurnSpeed);

        // Reach full aggressive rotation PID by 100 deg/s
        double interpolationScale = Math.min(1.0, Math.abs(filteredRobotTurnSpeed) / 100.0);

        double currentKp = kp + (kp_rotation - kp) * interpolationScale;
        double currentKd = kd + (kd_rotation - kd) * interpolationScale;

        double pTerm = currentKp * error;
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = currentKd * (error - lastError) / deltaTime;
        }

        // 2. DERIVATIVE SMOOTHING: Prevents "Derivative Kick" jitter
        smoothDTerm = (0.5 * dTerm) + (0.5 * smoothDTerm);

        // FEEDFORWARD: Counter-act robot rotation (using RAW speed for instant
        // response)
        double ffTerm = -rawRobotTurnVelocity * kv_head;

        // Dyanmic Peak Power Boost
        // Allow the motor to go up to 100% (1.0) when fighting high rotation speeds
        double maxAvailablePower = MAX_POWER + (1.0 - MAX_POWER) * interpolationScale;

        if (Math.abs(error) < angleTolerance) {
            power = Range.clip(ffTerm, -maxAvailablePower, maxAvailablePower);
        } else {
            // Use smoothed Derivative
            power = Range.clip(pTerm + smoothDTerm + ffTerm, -maxAvailablePower, maxAvailablePower);
        }

        // 5. SAFETY LIMITS & AGGRESSIVE SOFT STOP
        // Prevent hitting hard barriers even at full speed
        if (power > 0) {
            int distanceToLimit = MAX_TICKS - currentTicks;
            if (distanceToLimit <= 0) {
                power = 0; // Hard stop if at or past limit
            } else if (distanceToLimit < LIMIT_BUFFER) {
                // Exponentially reduce power as we approach the limit
                double scale = Math.pow((double) distanceToLimit / LIMIT_BUFFER, 2);
                power = Math.min(power, MAX_POWER * scale);
            }
        } else if (power < 0) {
            int distanceToLimit = currentTicks - MIN_TICKS;
            if (distanceToLimit <= 0) {
                power = 0; // Hard stop if at or past limit
            } else if (distanceToLimit < LIMIT_BUFFER) {
                // Exponentially reduce power as we approach the limit
                double scale = Math.pow((double) distanceToLimit / LIMIT_BUFFER, 2);
                power = Math.max(power, -MAX_POWER * scale);
            }
        }

        MotorTurela.setPower(power);
        lastError = error;

        // Update debug state variables
        currentAngle = currentTurretAngle;
        targetAngle = targetRelativeAngle;
        currentError = error;
    }

    /**
     * Calculates the angle the turret should have relative to the robot front
     * to point at the field target.
     */
    private double CalculateAngleToFieldTarget(double currentAngle) {
        double robotX = limeLight.GetRobotX();
        double robotY = limeLight.GetRobotY();
        double robotHeading = limeLight.GetRobotHeading(); // radians

        double targetX = limeLight.GetTargetX();
        double targetY = limeLight.GetTargetY();

        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double absoluteAngleToTarget = Math.atan2(deltaY, deltaX); // radians

        double relativeAngleRad = absoluteAngleToTarget - robotHeading;
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);

        while (relativeAngleDeg > 180)
            relativeAngleDeg -= 360;
        while (relativeAngleDeg < -180)
            relativeAngleDeg += 360;

        return relativeAngleDeg;
    }

    public void setTrackingTag(boolean track) {
        isTrackingTag = track;
    }

    public boolean isTrackingTag() {
        return isTrackingTag;
    }

    public double getCurrentDistance() {
        int currentTicks = MotorTurela.getCurrentPosition();
        double currentTurretAngle = currentTicks / TICKS_PER_DEGREE;
        return limeLight.GetDistance2DToAprilTagFromRobotCenter(currentTurretAngle);
    }

    public void SetKp(double newkp) {
        kp = newkp;
    }

    public void SetKd(double newkd) {
        kd = newkd;
    }

    public void SetKpRotation(double newkp) {
        kp_rotation = newkp;
    }

    public void SetKdRotation(double newkd) {
        kd_rotation = newkd;
    }

    public void SetKvHead(double newkv) {
        kv_head = newkv;
    }

    public double GetKp() {
        return kp;
    }

    public double GetKd() {
        return kd;
    }

    public double GetKpRotation() {
        return kp_rotation;
    }

    public double GetKdRotation() {
        return kd_rotation;
    }

    public double GetKvHead() {
        return kv_head;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public int getCurrentTicks() {
        return MotorTurela.getCurrentPosition();
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        return currentError;
    }

    public double getPower() {
        return power;
    }

    public void Run() {
        Update();
    }
}
