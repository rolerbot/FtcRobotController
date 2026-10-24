package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Turela care ocheste pe baza pozitiei din RobotPinpoint. Fara camera.
 *
 * Aceleasi valori si aceeasi logica ca vechiul TurretProfiledPIDControl (cel cu camera):
 * acelasi PID, profil, tickuri, deadzone si toleranta in functie de distanta.
 * Singura diferenta: in loc de "currentAngle - tx" (camera), unghiul tinta vine din
 * RobotPinpoint.GetAngleToGoalDeg(), iar distanta din RobotPinpoint.GetDistanceToGoalInches().
 *
 * RobotPinpoint.Run() trebuie apelat INAINTE de turret.Run() in bucla.
 *
 * Conventie: unghi turela pozitiv = spre STANGA (CCW). 0 = TICKS_AT_CENTER.
 *
 * Configurable (Panels): constantele public static se pot schimba LIVE din Panels (tab Configurables).
 * Schimbarile din Panels se pierd la restart -> dupa ce gasesti valorile bune, scrie-le aici.
 */
@Configurable
public class TurretPinpointPIDControl implements Subsystem {
    private DcMotorEx MotorTurela = null;
    private final RobotPinpoint robotPinpoint;
    private final GamepadEx ct2;
    private ButtonReader btnManual;
    private boolean isManualMode = false;

    // ---------- la fel ca in codul vechi ----------
    private final double TICKS_PER_DEGREE = 5.25;
    private final int TICKS_AT_CENTER = 455;
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

    private final ProfiledPIDController controller;

    private double persistentTargetAngle = 0;
    private double currentAngle = 0;
    private double targetAngle = 0;
    private int currentTicks = 0;
    private double neededAngle = 0;      // unghiul calculat din Pinpoint in bucla asta
    private double currentDeadzone = DEADZONE_FAR_DEG;
    private double currentTolerance = ANGLE_TOLERANCE_DEG;

    private boolean autoAim = true;

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
    }

    public void Update() {
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
        double distance = robotPinpoint.GetDistanceToGoalInches();
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
            neededAngle = normalizeDeg(robotPinpoint.GetAngleToGoalDeg() - TURRET_ZERO_HEADING_DEG);
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
        power = Range.clip(power, -maxTurretSpeed, maxTurretSpeed);

        if (currentTicks >= MAX_TICKS && power > 0)
            power = 0;
        if (currentTicks <= MIN_TICKS && power < 0)
            power = 0;

        MotorTurela.setPower(power);
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

    public double getCurrentAngle() { return currentAngle; }

    public int getCurrentTicks() { return MotorTurela.getCurrentPosition(); }

    public double getTargetAngle() { return targetAngle; }

    public void UpdateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== TURELA ===");
        telemetry.addData("  Mod", isManualMode ? "MANUAL" : (autoAim ? "AUTO (Pinpoint)" : "UNGHI FIX"));
        telemetry.addData("  Unghi", "%.1f°", currentAngle);
        telemetry.addData("  Tinta", "%.1f° (calculat %.1f°)", targetAngle, neededAngle);
        telemetry.addData("  Eroare (tinta-unghi)", "%.2f°", targetAngle - currentAngle);
        telemetry.addData("  Deadzone / toleranta", "%.1f° / %.1f°", currentDeadzone, currentTolerance);
        telemetry.addData("  Ticks", currentTicks);
        if (isTargetOutOfRange()) {
            telemetry.addLine("  Tinta in afara de +-90°!");
        }
    }

    public void Run() { Update(); }
}
