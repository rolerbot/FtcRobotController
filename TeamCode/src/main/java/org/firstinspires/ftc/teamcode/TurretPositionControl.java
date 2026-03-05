package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;

public class TurretPositionControl implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private LimeLight limeLight;
    private Shooter shooter;

    private final double TICKS_PER_DEGREE = 5.25;
    private final int TICKS_AT_CENTER = 472;
    private final int MIN_TICKS = 0;
    private final int MAX_TICKS = 915;

    private double maxTurretSpeed = 0.45; // 0.4
    private double avgDistance = 0;
    private double persistentTargetAngle = 0;
    private int targetTicks = TICKS_AT_CENTER;

    private final ElapsedTime lastTargetTimer = new ElapsedTime();
    private boolean isTrackingTag = true;
    private boolean hasTrackingLock = false;

    private final double VISION_TIMEOUT_SEC = 1.0;
    private final double ANGLE_TOLERANCE_DEG = 0.8;
    private final int TICK_TOLERANCE = (int) (ANGLE_TOLERANCE_DEG * TICKS_PER_DEGREE);

    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private boolean usePinpointFallback = true; // Disabled for now to ensure smooth vision tracking

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

        MotorTurela.setTargetPosition(MotorTurela.getCurrentPosition());
        MotorTurela.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        MotorTurela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        MotorTurela.setDirection(DcMotorEx.Direction.REVERSE);
        MotorTurela.setTargetPositionTolerance(TICK_TOLERANCE);
        MotorTurela.setPositionPIDFCoefficients(10.0); // P de 8 este mult mai safe pentru 30FPS
        MotorTurela.setPower(maxTurretSpeed);

        currentTicks = MotorTurela.getCurrentPosition();
        targetTicks = currentTicks;
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
        persistentTargetAngle = currentAngle;

        lastTargetTimer.reset();
    }

    public void Update() {
        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;

        if (isTrackingTag) {
            LLResult result = limeLight.getLimelight().getLatestResult();

            if (result != null) {
                double[] pythonOut = result.getPythonOutput();

                if (pythonOut != null && pythonOut.length >= 3 && pythonOut[0] > 0.5) {
                    lastTargetTimer.reset();
                    hasTrackingLock = true;

                    // USE pythonOut[1] (calculated horizontal angle from your script)
                    // Change to MINUS because if tag is at -10 deg (left),
                    // we need to INCREASE the turret angle to move left.
                    double horizontalAngleDeg = pythonOut[1];
                    double offsetDistance = pythonOut[2];

                    // DEADZONE marit la 1 grad pentru a opri oscilatiile fine
                    if (Math.abs(horizontalAngleDeg) > 1.0) {
                        persistentTargetAngle = currentAngle - horizontalAngleDeg;
                    }

                    double distInches = offsetDistance * 39.3701;
                    if (avgDistance == 0) {
                        avgDistance = distInches;
                    }
                    avgDistance = (0.85 * avgDistance) + (0.25 * distInches); // 70, 30

                    if (shooter != null) {
                        // shooter.SetCustomDistanceMeters(avgDistance * 0.0254);
                    }
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

        final double MAX_ANGLE_RANGE = 90.0;
        persistentTargetAngle = Range.clip(persistentTargetAngle, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);

        int newTargetTicks = (int) (persistentTargetAngle * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        newTargetTicks = Range.clip(newTargetTicks, MIN_TICKS, MAX_TICKS);

        // Only update motor if target shifted significantly to avoid jitter/stalling
        if (Math.abs(newTargetTicks - targetTicks) >= 1) {
            targetTicks = newTargetTicks;
            MotorTurela.setTargetPosition(targetTicks);
        }

        // Ensure motor always has power to reach target
        MotorTurela.setPower(maxTurretSpeed);
        targetAngle = persistentTargetAngle;
    }

    public boolean isOnTarget() {
        int error = Math.abs(targetTicks - currentTicks);
        return error <= TICK_TOLERANCE;
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

        double robotX = limeLight.GetRobotX();
        double robotY = limeLight.GetRobotY();
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
        return targetTicks - currentTicks;
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
        targetTicks = (int) (persistentTargetAngle * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        MotorTurela.setTargetPosition(targetTicks);
    }

    public void setTargetTicks(int ticks) {
        targetTicks = ticks;
        persistentTargetAngle = (targetTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;

        if (persistentTargetAngle > 90.0) {
            persistentTargetAngle = 90.0;
            targetTicks = (int) (90.0 * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        } else if (persistentTargetAngle < -90.0) {
            persistentTargetAngle = -90.0;
            targetTicks = (int) (-90.0 * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        }

        MotorTurela.setTargetPosition(targetTicks);
    }

    public void setMaxSpeed(double speed) {
        maxTurretSpeed = Range.clip(speed, 0.0, 1.0);
        MotorTurela.setPower(maxTurretSpeed);
    }

    public double getMaxSpeed() {
        return maxTurretSpeed;
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

    public void Run() {
        Update();
    }
}