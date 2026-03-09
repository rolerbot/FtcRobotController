package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;

public class TurretProfiledPIDControl implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private final LimeLight limeLight;
    private final Shooter shooter;

    private final double TICKS_PER_DEGREE = 5.25;
    private final int TICKS_AT_CENTER = 472;
    private final int MIN_TICKS = 0;
    private final int MAX_TICKS = 915;

    // Profiled PID Constants - These will need tuning
    public static double Kp = 0.043; // (0.051 * 0.8)
    public static double Ki = 0.00;
    public static double Kd = 0.005; // (0.0045 * 1.4)
    public static double maxVelocity = 300.0; // degrees per second
    public static double maxAcceleration = 420.0; // (600.0 * 0.6) // 360

    private ProfiledPIDController controller;

    private double maxTurretSpeed = 0.8;
    private double avgDistance = 0;
    private double persistentTargetAngle = 0;

    private final ElapsedTime lastTargetTimer = new ElapsedTime();
    private boolean isTrackingTag = true;
    private boolean hasTrackingLock = false;

    private final double VISION_TIMEOUT_SEC = .3;
    private final double ANGLE_TOLERANCE_DEG = .9;

    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private boolean usePinpointFallback = true;

    private double lastKp = Kp, lastKi = Ki, lastKd = Kd;

    public TurretProfiledPIDControl(LimeLight limeLight, Shooter shooter) {
        this.limeLight = limeLight;
        this.shooter = shooter;
        setupController();
    }

    public TurretProfiledPIDControl(LimeLight limeLight) {
        this.limeLight = limeLight;
        this.shooter = null;
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
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
        persistentTargetAngle = currentAngle;

        // Reset controller to current state
        controller.reset(currentAngle);
        targetAngle = currentAngle;

        lastTargetTimer.reset();
    }

    public void Update() {
        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;

        // --- 1. Calculate Dynamic Thresholds & PID ---
        double actingKp = Kp;
        double actingKd = Kd;
        double actingMaxAccel = maxAcceleration;
        double currentDeadzone = 0.5;
        double currentTolerance = ANGLE_TOLERANCE_DEG;

        // Dynamic Thresholds to stop oscillation at closer distances
        if (avgDistance > 0) {
            if (avgDistance < 90.55) { // < 2.3m (Very Close)
                currentDeadzone = 1.3;
                currentTolerance = 1.5;
            } else if (avgDistance < 130.0) { // < 3.3m (Mid-Range)
                currentDeadzone = 0.8;
                currentTolerance = 1.0;
            }
        }

        // --- PINPOINT FALLBACK BOOST ---
        // If we are NOT actively tracking a tag, we can be more aggressive to find it
        // faster
        boolean isFallbackMode = !hasTrackingLock || lastTargetTimer.seconds() >= VISION_TIMEOUT_SEC || !isTrackingTag;
        double actingMaxSpeed = maxTurretSpeed;
        double actingMaxVel = maxVelocity;

        if (isFallbackMode) {
            actingMaxSpeed = 0.8; // Full motor power for pinpoint moves
            actingMaxVel = maxVelocity; // 25% faster velocity
            actingMaxAccel = maxAcceleration; // 25% faster acceleration
        }

        // Keep last values updated if they change from tuning
        if (Kp != lastKp || Ki != lastKi || Kd != lastKd) {
            lastKp = Kp;
            lastKi = Ki;
            lastKd = Kd;
        }

        controller.setPID(actingKp, Ki, actingKd);
        controller.setConstraints(new TrapezoidProfile.Constraints(actingMaxVel, actingMaxAccel));
        controller.setTolerance(currentTolerance);

        // --- 2. Process Vision Tracking ---
        if (isTrackingTag) {
            LLResult result = limeLight.getLimelight().getLatestResult();

            if (result != null) {
                double[] pythonOut = result.getPythonOutput();

                if (pythonOut != null && pythonOut.length >= 3 && pythonOut[0] > 0.5) {
                    lastTargetTimer.reset();
                    hasTrackingLock = true;

                    double horizontalAngleDeg = pythonOut[1];
                    double offsetDistance = pythonOut[2];

                    if (Math.abs(horizontalAngleDeg) > currentDeadzone) {
                        persistentTargetAngle = currentAngle - horizontalAngleDeg;
                    }

                    double distInches = offsetDistance * 39.3701;
                    if (avgDistance == 0) {
                        avgDistance = distInches;
                    }
                    // Fixed EMA weights (0.75 + 0.25 = 1.0)
                    avgDistance = (0.75 * avgDistance) + (0.25 * distInches);
                } else {
                    if (hasTrackingLock && usePinpointFallback &&
                            lastTargetTimer.seconds() >= VISION_TIMEOUT_SEC) {
                        persistentTargetAngle = getClosestAngleByPinpoint();
                    }
                }
            }
        } else {
            if (usePinpointFallback) {
                persistentTargetAngle = getClosestAngleByPinpoint();
            }
        }

        // --- 3. Execute PID Control ---
        final double MAX_ANGLE_RANGE = 90.0;
        persistentTargetAngle = Range.clip(persistentTargetAngle, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
        targetAngle = persistentTargetAngle;

        // Calculate PID output using the profiled controller
        double power = controller.calculate(currentAngle, targetAngle);

        // Clip power to the active max speed
        power = Range.clip(power, -actingMaxSpeed, actingMaxSpeed);

        // Safety bounds
        if (currentTicks >= MAX_TICKS && power > 0)
            power = 0;
        if (currentTicks <= MIN_TICKS && power < 0)
            power = 0;

        MotorTurela.setPower(power);
    }

    public boolean isOnTarget() {
        return controller.atGoal();
    }

    private double getClosestAngleByPinpoint() {
        double TARGET_X;
        double TARGET_Y;

        if (limeLight.IsBlue()) {
            TARGET_X = 0.0;
            TARGET_Y = 144.0;
        } else {
            TARGET_X = 144.0;
            TARGET_Y = 144.0;
        }

        double robotX = limeLight.GetRobotX() + 2.11;
        double robotY = limeLight.GetRobotY() - 3.31;
        double robotHeadingRad = limeLight.GetRobotHeading();
        double robotHeading = Math.toDegrees(robotHeadingRad);

        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;

        double angleToTargetRadians = Math.atan2(deltaY, deltaX);
        double angleToTargetDegrees = Math.toDegrees(angleToTargetRadians);

        double relativeAngle = angleToTargetDegrees - robotHeading;

        while (relativeAngle > 180)
            relativeAngle -= 360;
        while (relativeAngle < -180)
            relativeAngle += 360;

        return relativeAngle;
    }

    public double getErrorDegrees() {
        return targetAngle - currentAngle;
    }

    public int getErrorTicks() {
        return (int) (targetAngle * TICKS_PER_DEGREE + TICKS_AT_CENTER) - currentTicks;
    }

    public void setTrackingTag(boolean track) {
        isTrackingTag = track;
    }

    public boolean isTrackingTag() {
        return isTrackingTag;
    }

    public void setUsePinpointFallback(boolean use) {
        usePinpointFallback = use;
    }

    public boolean isUsingPinpointFallback() {
        return usePinpointFallback;
    }

    public double getCurrentDistance() {
        return avgDistance;
    }

    public void setTargetAngle(double angleDegrees) {
        persistentTargetAngle = Range.clip(angleDegrees, -90.0, 90.0);
    }

    public void setTargetTicks(int ticks) {
        persistentTargetAngle = (ticks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
        persistentTargetAngle = Range.clip(persistentTargetAngle, -90.0, 90.0);
    }

    public void setMaxSpeed(double speed) {
        maxTurretSpeed = Range.clip(speed, 0.0, 1.0);
    }

    public double getMaxSpeed() {
        return maxTurretSpeed;
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

    public int getTargetTicks() {
        return (int) (targetAngle * TICKS_PER_DEGREE + TICKS_AT_CENTER);
    }

    public boolean hasTrackingLock() {
        return hasTrackingLock;
    }

    public double getTimeSinceLastTag() {
        return lastTargetTimer.seconds();
    }

    public boolean isActivelyTracking() {
        return hasTrackingLock && lastTargetTimer.seconds() < VISION_TIMEOUT_SEC;
    }

    public void resetTrackingLock() {
        hasTrackingLock = false;
        lastTargetTimer.reset();
    }

    public void updatePID(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
        controller.setPID(p, i, d);
    }

    public void updateConstraints(double maxVel, double maxAccel) {
        maxVelocity = maxVel;
        maxAcceleration = maxAccel;
        controller.setConstraints(new TrapezoidProfile.Constraints(maxVel, maxAccel));
    }

    public void Run() {
        Update();
    }
}
