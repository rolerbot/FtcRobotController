package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class TurretPositionControl implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private LimeLight limeLight;
    private Shooter shooter;
    private GamepadEx ct2;
    private ButtonReader btnManual;
    private boolean isManualMode = false;

    private final double TICKS_PER_DEGREE = 5.25;
    private final int TICKS_AT_CENTER = 472;
    private final int MIN_TICKS = 0;
    private final int MAX_TICKS = 915;

    private double maxTurretSpeed = 0.7;
    private double avgDistance = 0;
    private double persistentTargetAngle = 0;
    private int targetTicks = TICKS_AT_CENTER;

    private final ElapsedTime lastTargetTimer = new ElapsedTime();
    private boolean isTrackingTag = true;
    private boolean hasTrackingLock = false;

    private final double VISION_TIMEOUT_SEC = 1.0;
    private final double ANGLE_TOLERANCE_DEG = 0.9;
    private final int TICK_TOLERANCE = (int) (ANGLE_TOLERANCE_DEG * TICKS_PER_DEGREE);

    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private boolean usePinpointFallback = true;

    public TurretPositionControl(LimeLight limeLight, Shooter shooter, GamepadEx ct2) {
        this.limeLight = limeLight;
        this.shooter = shooter;
        this.ct2 = ct2;
        if (ct2 != null) {
            this.btnManual = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);
        }
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
        MotorTurela.setPositionPIDFCoefficients(8.0);
        MotorTurela.setPower(maxTurretSpeed);

        currentTicks = MotorTurela.getCurrentPosition();
        targetTicks = currentTicks;
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
        persistentTargetAngle = currentAngle;

        lastTargetTimer.reset();
    }

    public void Update() {
        if (ct2 != null) {
            btnManual.readValue();
            if (btnManual.wasJustPressed()) {
                isManualMode = !isManualMode;
                if (isManualMode) {
                    MotorTurela.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    // Reset: tell it this is the new zero (center)
                    MotorTurela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    // Now current is 0. If nominal center is 472, we move to 472.
                    // But if the user wants this to be the NEW CENTER, we move to TICKS_AT_CENTER
                    MotorTurela.setTargetPosition(TICKS_AT_CENTER);
                    MotorTurela.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    persistentTargetAngle = 0;
                }
            }
        }

        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;

        if (isManualMode) {
            double stick = ct2.getLeftX();
            if (Math.abs(stick) < 0.05)
                stick = 0;
            double manualPower = -stick * 0.7; // Higher max speed for "sensibilitate"
            // Bound checking in manual
            if (currentTicks >= MAX_TICKS && manualPower > 0)
                manualPower = 0;
            if (currentTicks <= MIN_TICKS && manualPower < 0)
                manualPower = 0;
            MotorTurela.setPower(manualPower);
            return;
        }

        if (isTrackingTag) {
            LLResult result = limeLight.getLimelight().getLatestResult();
            if (result != null) {
                double[] pythonOut = result.getPythonOutput();
                if (pythonOut != null && pythonOut.length >= 3 && pythonOut[0] > 0.5) {
                    lastTargetTimer.reset();
                    hasTrackingLock = true;
                    double horizontalAngleDeg = pythonOut[1];
                    double offsetDistance = pythonOut[2];
                    if (Math.abs(horizontalAngleDeg) > .3) {
                        persistentTargetAngle = currentAngle - horizontalAngleDeg;
                    }
                    double distInches = offsetDistance * 39.3701;
                    if (avgDistance == 0)
                        avgDistance = distInches;
                    avgDistance = (0.85 * avgDistance) + (0.15 * distInches);
                } else {
                    if (hasTrackingLock && usePinpointFallback && lastTargetTimer.seconds() >= VISION_TIMEOUT_SEC) {
                        persistentTargetAngle = getClosestAngleByPinpoint();
                    }
                }
            }
        } else if (usePinpointFallback) {
            persistentTargetAngle = getClosestAngleByPinpoint();
        }

        final double MAX_ANGLE_RANGE = 90.0;
        persistentTargetAngle = Range.clip(persistentTargetAngle, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);

        int newTargetTicks = (int) (persistentTargetAngle * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        newTargetTicks = Range.clip(newTargetTicks, MIN_TICKS, MAX_TICKS);

        if (Math.abs(newTargetTicks - targetTicks) >= 1) {
            targetTicks = newTargetTicks;
            MotorTurela.setTargetPosition(targetTicks);
        }
        MotorTurela.setPower(maxTurretSpeed);
        targetAngle = persistentTargetAngle;
    }

    public boolean isOnTarget() {
        return Math.abs(targetTicks - currentTicks) <= TICK_TOLERANCE;
    }

    private double getClosestAngleByPinpoint() {
        double TARGET_X = limeLight.IsBlue() ? 0.0 : 144.0;
        double TARGET_Y = 144.0;
        double robotX = limeLight.GetRobotX() + 2.11;
        double robotY = limeLight.GetRobotY() - 3.31;
        double robotHeading = Math.toDegrees(limeLight.GetRobotHeading());

        double deltaX = TARGET_X - robotX;
        double deltaY = TARGET_Y - robotY;
        double angleToTargetDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double relativeAngle = angleToTargetDegrees - robotHeading;

        while (relativeAngle > 180)
            relativeAngle -= 360;
        while (relativeAngle < -180)
            relativeAngle += 360;
        return relativeAngle;
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

    public double getCurrentDistance() {
        return avgDistance;
    }

    public void setTargetAngle(double angleDegrees) {
        persistentTargetAngle = Range.clip(angleDegrees, -90.0, 90.0);
        targetTicks = (int) (persistentTargetAngle * TICKS_PER_DEGREE + TICKS_AT_CENTER);
        MotorTurela.setTargetPosition(targetTicks);
    }

    public void setTargetTicks(int ticks) {
        targetTicks = Range.clip(ticks, MIN_TICKS, MAX_TICKS);
        persistentTargetAngle = (targetTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
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
        return MotorTurela.getCurrentPosition();
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