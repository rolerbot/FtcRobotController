package org.firstinspires.ftc.teamcode;

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
 * Turela care ocheste DOAR pe baza pozitiei din RobotPinpoint. Fara camera.
 *
 * In fiecare bucla:
 *  1. ia unghiul spre goal calculat de RobotPinpoint (din axa turelei, fata de fata robotului)
 *  2. ProfiledPID pe unghi + feedforward pe viteza cu care se misca tinta
 *
 * RobotPinpoint.Run() trebuie apelat INAINTE de turret.Run() in bucla.
 *
 * Conventie: unghi turela pozitiv = spre STANGA (CCW). 0 = TICKS_AT_CENTER.
 */
public class TurretPinpointPIDControl implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private final RobotPinpoint robotPinpoint;
    private final GamepadEx ct2;
    private ButtonReader btnManual;
    private boolean isManualMode = false;

    public static String TURRET_MOTOR_NAME = "MotorTurela";

    private final double TICKS_PER_DEGREE = 5.25;
    private final int TICKS_AT_CENTER = 455;
    private final int MIN_TICKS = 0;
    private final int MAX_TICKS = 910;
    private final double MAX_ANGLE_RANGE = 90.0;

    // Unde priveste turela la TICKS_AT_CENTER fata de fata robotului (0 = inainte, 180 = inapoi)
    public static double TURRET_ZERO_HEADING_DEG = 0.0;

    public static double Kp = 0.042;
    public static double Ki = 0.0;
    public static double Kd = 0.005;
    public static double maxVelocity = 300.0;      // grade/s
    public static double maxAcceleration = 400.0;  // grade/s^2

    // Feedforward: putere per (grad/s) de miscare a tintei.
    // Masoara viteza turelei la putere 1.0 (grade/s), porneste cu kV = 1 / viteza aceea.
    // Lasa 0 pana cand PID-ul merge bine cu robotul nemiscat.
    public static double kV = 0.0;
    public static double TARGET_RATE_FILTER = 0.3; // 0..1, netezirea vitezei tintei

    public static double ANGLE_TOLERANCE_DEG = 1.0;

    private final ProfiledPIDController controller;
    private double maxTurretSpeed = 0.8;

    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;

    private boolean autoAim = true;
    private double manualTargetAngle = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastRawTarget = Double.NaN;
    private double targetRate = 0; // grade/s

    /** ct2 poate fi null (autonomie) - atunci nu exista mod manual. */
    public TurretPinpointPIDControl(RobotPinpoint robotPinpoint, GamepadEx ct2) {
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
        MotorTurela = hwMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
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
        targetAngle = currentAngle;
        manualTargetAngle = currentAngle;
        controller.reset(currentAngle);
        lastRawTarget = Double.NaN;
        targetRate = 0;
        loopTimer.reset();
    }

    public void Update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (ct2 != null) {
            btnManual.readValue();
            if (btnManual.wasJustPressed()) {
                isManualMode = !isManualMode;
                if (!isManualMode) {
                    // ATENTIE: 0 tickuri = MIN_TICKS (capatul din dreapta).
                    // Apasa doar cu turela impinsa la capatul MIN.
                    MotorTurela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    MotorTurela.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    controller.reset(ticksToAngle(0));
                    lastRawTarget = Double.NaN;
                    targetRate = 0;
                }
            }
        }

        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = ticksToAngle(currentTicks);

        if (isManualMode) {
            double stick = ct2.getLeftX();
            if (Math.abs(stick) < 0.05) stick = 0;
            double manualPower = -stick * 0.7;
            if (currentTicks >= MAX_TICKS && manualPower > 0) manualPower = 0;
            if (currentTicks <= MIN_TICKS && manualPower < 0) manualPower = 0;
            MotorTurela.setPower(manualPower);
            return;
        }

        // ---------- tinta ----------
        double rawTarget = autoAim ? computeAngleToGoal() : manualTargetAngle;

        if (!Double.isNaN(lastRawTarget) && dt > 1e-4) {
            double rate = normalizeDeg(rawTarget - lastRawTarget) / dt;
            targetRate += TARGET_RATE_FILTER * (rate - targetRate);
        }
        lastRawTarget = rawTarget;

        targetAngle = Range.clip(rawTarget, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
        boolean atLimit = Math.abs(rawTarget) >= MAX_ANGLE_RANGE;

        // ---------- control ----------
        controller.setPID(Kp, Ki, Kd);
        controller.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        controller.setTolerance(ANGLE_TOLERANCE_DEG);

        double power = controller.calculate(currentAngle, targetAngle);
        if (autoAim && !atLimit) {
            power += kV * targetRate;
        }
        power = Range.clip(power, -maxTurretSpeed, maxTurretSpeed);

        if (currentTicks >= MAX_TICKS && power > 0) power = 0;
        if (currentTicks <= MIN_TICKS && power < 0) power = 0;

        MotorTurela.setPower(power);
    }

    /** Unghiul (fata de centrul turelei) la care trebuie sa stea turela ca sa ocheasca goal-ul. */
    private double computeAngleToGoal() {
        return normalizeDeg(robotPinpoint.GetAngleToGoalDeg() - TURRET_ZERO_HEADING_DEG);
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

    /** true = ocheste automat goal-ul; false = sta la unghiul dat cu setTargetAngle(). */
    public void setAutoAim(boolean enabled) {
        if (enabled && !autoAim) {
            lastRawTarget = Double.NaN;
            targetRate = 0;
        }
        autoAim = enabled;
    }

    public boolean isAutoAim() { return autoAim; }

    /** Unghi fix (grade, + = stanga). Opreste ochirea automata. */
    public void setTargetAngle(double angleDegrees) {
        manualTargetAngle = Range.clip(angleDegrees, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
        autoAim = false;
    }

    /** Distanta axa turelei -> goal, in inch. */
    public double getDistanceToGoal() { return robotPinpoint.GetDistanceToGoalInches(); }

    /** true daca goal-ul e in afara celor +-90 de grade (turela sta la limita). */
    public boolean isTargetOutOfRange() {
        return autoAim && !Double.isNaN(lastRawTarget) && Math.abs(lastRawTarget) > MAX_ANGLE_RANGE;
    }

    public boolean isManualMode() { return isManualMode; }

    public void setMaxSpeed(double speed) { maxTurretSpeed = Range.clip(speed, 0.0, 1.0); }

    public double getCurrentAngle() { return currentAngle; }

    public int getCurrentTicks() { return MotorTurela.getCurrentPosition(); }

    public double getTargetAngle() { return targetAngle; }

    public double getTargetRate() { return targetRate; }

    public void UpdateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== TURELA ===");
        telemetry.addData("  Mod", isManualMode ? "MANUAL" : (autoAim ? "AUTO (Pinpoint)" : "UNGHI FIX"));
        telemetry.addData("  Unghi", "%.1f° (tinta %.1f°)", currentAngle, targetAngle);
        telemetry.addData("  Ticks", currentTicks);
        if (isTargetOutOfRange()) {
            telemetry.addLine("  Goal in afara de +-90°!");
        }
    }

    public void Run() { Update(); }
}
