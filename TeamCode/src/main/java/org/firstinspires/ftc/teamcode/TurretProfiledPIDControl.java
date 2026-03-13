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

public class TurretProfiledPIDControl implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private final LimeLight limeLight;
    private final Shooter shooter;
    private GamepadEx ct2;
    private ButtonReader btnManual;
    private boolean isManualMode = false;

    private final double TICKS_PER_DEGREE = 5.25;
    private final int TICKS_AT_CENTER = 472;
    private final int MIN_TICKS = 0;
    private final int MAX_TICKS = 915;

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

    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private boolean usePinpointFallback = true;
    private double lastKp = Kp, lastKi = Ki, lastKd = Kd;

    public TurretProfiledPIDControl(LimeLight limeLight, Shooter shooter, GamepadEx ct2) {
        this.limeLight = limeLight;
        this.shooter = shooter;
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
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;
        persistentTargetAngle = currentAngle;

        controller.reset(currentAngle);
        targetAngle = currentAngle;
        lastTargetTimer.reset();
    }

    public void Update() {
        if (ct2 != null) {
            btnManual.readValue();
            if (btnManual.wasJustPressed()) {
                isManualMode = !isManualMode;
                if (!isManualMode) {
                    MotorTurela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    MotorTurela.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    persistentTargetAngle = 0;
                    controller.reset(0);
                }
            }
        }

        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = (currentTicks - TICKS_AT_CENTER) / TICKS_PER_DEGREE;

        if (isManualMode) {
            double stick = ct2.getLeftX();
            if (Math.abs(stick) < 0.05) stick = 0;
            double manualPower = -stick * 0.7; // Higher max speed for "sensibilitate"
            // Bound checking in manual
            if (currentTicks >= MAX_TICKS && manualPower > 0) manualPower = 0;
            if (currentTicks <= MIN_TICKS && manualPower < 0) manualPower = 0;
            MotorTurela.setPower(manualPower);
            return;
        }

        double actingKp = Kp;
        double actingKd = Kd;
        double actingMaxAccel = maxAcceleration;
        double currentDeadzone = 0.5;
        double currentTolerance = ANGLE_TOLERANCE_DEG;

        if (avgDistance > 0) {
            if (avgDistance < 90.55) { // < 2.3m (Very Close)
                currentDeadzone = 1.8; // Wider deadzone for stability
                currentTolerance = 2.0; // Higher tolerance to stop hunting
            } else if (avgDistance < 130.0) { // < 3.3m (Mid-Range)
                currentDeadzone = 2;
                currentTolerance = 2.2;
            }
        }

        boolean isFallbackMode = !hasTrackingLock || lastTargetTimer.seconds() >= VISION_TIMEOUT_SEC || !isTrackingTag;
        double actingMaxSpeed = maxTurretSpeed;
        double actingMaxVel = maxVelocity;

        if (isFallbackMode) {
            actingMaxSpeed = 0.8;
            actingMaxVel = maxVelocity;
            actingMaxAccel = maxAcceleration;
        }

        if (Kp != lastKp || Ki != lastKi || Kd != lastKd) {
            lastKp = Kp; lastKi = Ki; lastKd = Kd;
        }

        controller.setPID(actingKp, Ki, actingKd);
        controller.setConstraints(new TrapezoidProfile.Constraints(actingMaxVel, actingMaxAccel));
        controller.setTolerance(currentTolerance);

        if (isTrackingTag) {
            LLResult result = limeLight.getLimelight().getLatestResult();
            if (result != null) {
                double[] pythonOut = result.getPythonOutput();
                if (pythonOut != null && pythonOut.length >= 3 && pythonOut[0] > 0.5) {
                    lastTargetTimer.reset();
                    hasTrackingLock = true;
                    double horizontalAngleDeg = limeLight.getLimelight().getLatestResult().getTx();//pythonOut[1];
                    double offsetDistance = pythonOut[2];
                    if (Math.abs(horizontalAngleDeg) > currentDeadzone) {
                        persistentTargetAngle = currentAngle - horizontalAngleDeg;
                    }
                    double distInches = offsetDistance * 39.3701;
                    if (avgDistance == 0) avgDistance = distInches;
                    avgDistance = (0.75 * avgDistance) + (0.25 * distInches);
                } else if (hasTrackingLock && usePinpointFallback && lastTargetTimer.seconds() >= VISION_TIMEOUT_SEC) {
                    persistentTargetAngle = getClosestAngleByPinpoint();
                }
            }
        } else if (usePinpointFallback) {
            persistentTargetAngle = getClosestAngleByPinpoint();
        }

        final double MAX_ANGLE_RANGE = 90.0;
        persistentTargetAngle = Range.clip(persistentTargetAngle, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
        targetAngle = persistentTargetAngle;

        double power = controller.calculate(currentAngle, targetAngle);
        power = Range.clip(power, -actingMaxSpeed, actingMaxSpeed);

        if (currentTicks >= MAX_TICKS && power > 0) power = 0;
        if (currentTicks <= MIN_TICKS && power < 0) power = 0;

        MotorTurela.setPower(power);
    }

    public boolean isOnTarget() { return controller.atGoal(); }

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

        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;
        return relativeAngle;
    }

    public void setTrackingTag(boolean track) { isTrackingTag = track; }
    public boolean isTrackingTag() { return isTrackingTag; }
    public double getCurrentDistance() { return avgDistance; }
    public void setTargetAngle(double angleDegrees) { persistentTargetAngle = Range.clip(angleDegrees, -90.0, 90.0); }
    public void setMaxSpeed(double speed) { maxTurretSpeed = Range.clip(speed, 0.0, 1.0); }
    public double getCurrentAngle() { return currentAngle; }
    public int getCurrentTicks() { return MotorTurela.getCurrentPosition(); }
    public double getTargetAngle() { return targetAngle; }
    public void resetTrackingLock() { hasTrackingLock = false; lastTargetTimer.reset(); }
    public void Run() { Update(); }
}
