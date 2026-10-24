package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Turela care ocheste DOAR pe baza goBILDA Pinpoint. Fara camera, fara Limelight.
 *
 * In fiecare bucla:
 *  1. citeste X, Y, heading din Pinpoint
 *  2. calculeaza pozitia axei turelei pe teren (offset rotit cu heading-ul)
 *  3. unghiul spre goal fata de robot = unghiul la care trebuie sa stea turela
 *  4. ProfiledPID pe unghi + feedforward pe viteza cu care se schimba tinta
 *
 * Conventie: unghi turela pozitiv = spre STANGA (CCW), ca in TurretProfiledPIDControl.
 *
 * Pinpoint da pozitia fata de locul unde a fost resetat. Ca GOAL_X/GOAL_Y sa aiba sens,
 * apeleaza setStartPose(...) la init cu pozitia reala de start pe teren
 * (sau seteaza pozitia din alt cod, ex. din follower-ul de autonomie).
 */
public class TurretPinpointPIDControl implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private GoBildaPinpointDriver pinpoint = null;
    private GamepadEx ct2;
    private ButtonReader btnManual;
    private boolean isManualMode = false;

    // numele din configuratia robotului
    public static String PINPOINT_NAME = "pinpoint";
    public static String TURRET_MOTOR_NAME = "MotorTurela";

    private final double TICKS_PER_DEGREE = 5.25;
    private final int TICKS_AT_CENTER = 455;
    private final int MIN_TICKS = 0;
    private final int MAX_TICKS = 910;
    private final double MAX_ANGLE_RANGE = 90.0;

    // ===================== DE MASURAT / SETAT (inch) =====================
    // Axa turelei fata de centrul de tracking al Pinpoint, in sistemul ROBOTULUI
    public static double PIVOT_FWD_IN = 0.0;   // + = spre fata robotului
    public static double PIVOT_LEFT_IN = 0.0;  // + = spre stanga robotului
    // Unde priveste turela la TICKS_AT_CENTER fata de fata robotului (0 = inainte, 180 = inapoi)
    public static double TURRET_ZERO_HEADING_DEG = 0.0;

    // Punctul de ochire pe teren, in acelasi sistem de coordonate ca Pinpoint
    public static double GOAL_X_BLUE = 0.0;
    public static double GOAL_X_RED = 144.0;
    public static double GOAL_Y = 144.0;
    // =====================================================================

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

    private ProfiledPIDController controller;
    private double maxTurretSpeed = 0.8;
    private boolean isBlue;

    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private double distanceToGoal = 0;

    private double robotX = 0, robotY = 0, robotHeadingDeg = 0;

    private boolean autoAim = true;
    private double manualTargetAngle = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastRawTarget = Double.NaN;
    private double targetRate = 0; // grade/s

    /** Pinpoint-ul e luat din hardwareMap la Initialize (dupa PINPOINT_NAME). */
    public TurretPinpointPIDControl(boolean isBlue, GamepadEx ct2) {
        this.isBlue = isBlue;
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
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
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

    /** Seteaza pozitia de start a robotului pe teren (inch, grade). */
    public void setStartPose(double xIn, double yIn, double headingDeg) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg));
        lastRawTarget = Double.NaN;
    }

    public void Update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        pinpoint.update();
        robotX = pinpoint.getPosX(DistanceUnit.INCH);
        robotY = pinpoint.getPosY(DistanceUnit.INCH);
        robotHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);

        if (ct2 != null) {
            btnManual.readValue();
            if (btnManual.wasJustPressed()) {
                isManualMode = !isManualMode;
                if (!isManualMode) {
                    // ATENTIE: 0 tickuri = MIN_TICKS. Apasa doar cu turela la capatul MIN.
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

        // ---------- tinta din Pinpoint ----------
        double angleToGoal = computeAngleToGoal(); // calculeaza si distanta
        double rawTarget = autoAim ? angleToGoal : manualTargetAngle;

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
        if (!atLimit) {
            power += kV * targetRate;
        }
        power = Range.clip(power, -maxTurretSpeed, maxTurretSpeed);

        if (currentTicks >= MAX_TICKS && power > 0) power = 0;
        if (currentTicks <= MIN_TICKS && power < 0) power = 0;

        MotorTurela.setPower(power);
    }

    /** Unghiul (fata de centrul turelei) la care trebuie sa stea turela ca sa ocheasca goal-ul. */
    private double computeAngleToGoal() {
        double h = Math.toRadians(robotHeadingDeg);
        double cos = Math.cos(h), sin = Math.sin(h);

        // pozitia axei turelei pe teren
        double pivotX = robotX + PIVOT_FWD_IN * cos - PIVOT_LEFT_IN * sin;
        double pivotY = robotY + PIVOT_FWD_IN * sin + PIVOT_LEFT_IN * cos;

        double goalX = isBlue ? GOAL_X_BLUE : GOAL_X_RED;
        double dx = goalX - pivotX;
        double dy = GOAL_Y - pivotY;
        distanceToGoal = Math.hypot(dx, dy);

        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        return normalizeDeg(fieldAngle - robotHeadingDeg - TURRET_ZERO_HEADING_DEG);
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

    public void setAlliance(boolean blue) { isBlue = blue; }

    /** true = ocheste automat goal-ul; false = sta la unghiul dat cu setTargetAngle(). */
    public void setAutoAim(boolean enabled) { autoAim = enabled; }

    public boolean isAutoAim() { return autoAim; }

    public void setTargetAngle(double angleDegrees) {
        manualTargetAngle = Range.clip(angleDegrees, -MAX_ANGLE_RANGE, MAX_ANGLE_RANGE);
        autoAim = false;
    }

    /** Distanta axa turelei -> goal, in inch (pentru shooter). */
    public double getDistanceToGoal() { return distanceToGoal; }

    /** Compatibil cu TurretProfiledPIDControl. */
    public double getCurrentDistance() { return distanceToGoal; }

    /** true daca goal-ul e in afara celor +-90 de grade (turela sta la limita). */
    public boolean isTargetOutOfRange() {
        return !Double.isNaN(lastRawTarget) && Math.abs(lastRawTarget) > MAX_ANGLE_RANGE;
    }

    public void setMaxSpeed(double speed) { maxTurretSpeed = Range.clip(speed, 0.0, 1.0); }

    public double getCurrentAngle() { return currentAngle; }

    public int getCurrentTicks() { return MotorTurela.getCurrentPosition(); }

    public double getTargetAngle() { return targetAngle; }

    public double getTargetRate() { return targetRate; }

    public double getRobotX() { return robotX; }

    public double getRobotY() { return robotY; }

    public double getRobotHeadingDeg() { return robotHeadingDeg; }

    public void Run() { Update(); }
}