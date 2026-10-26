package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Turela cu un motor si encoder. Un PID cu profil tine turela la unghiul tinta.
//
// Unghi: 0 = turela centrata (pozitia din care porneste robotul), + = stanga, - = dreapta.
// IMPORTANT: la pornirea programului turela trebuie sa fie CENTRATA, pentru ca encoderul se reseteaza la 0.
//
// Controale:
//  Stick stanga (stanga/dreapta) - muta unghiul tinta
//  Apasare stick stanga          - turela inapoi la centru (0 grade)
//  Apasare stick dreapta         - seteaza pozitia curenta ca 0 (reset encoder)
//
// Constantele public static se pot schimba LIVE din Panels (tab Configurables).
// Schimbarile din Panels se pierd la restart: scrie aici valorile bune.
@Configurable
public class Turret implements Subsystem {
    private DcMotorEx MotorTurela;
    private final GamepadEx ct;
    private final Telemetry telemetry;

    ButtonReader Center, ResetEncoder;

    // MASOARA pe robotul vostru: roteste turela de mana de la un capat la altul,
    // citeste ticks din telemetrie si imparte la grade. 5.1 e valoarea de la robotul vechi.
    public static double TICKS_PER_DEGREE = 5.1;
    public static double MIN_ANGLE = -90;
    public static double MAX_ANGLE = 90;

    public static double Kp = 0.032;
    public static double Ki = 0.0;
    public static double Kd = 0.005;
    public static double MaxVelocity = 300.0;      // grade/s
    public static double MaxAcceleration = 200.0;  // grade/s^2
    public static double MaxPower = 0.6;
    // Putere mica adaugata ca sa invinga frecarea statica; 0 = oprit
    public static double kS = 0.0;
    public static double ToleranceDeg = 1.0;

    // Cat de repede se muta tinta cand impingi stick-ul complet
    public static double ManualSpeedDegPerSec = 90;

    private final ProfiledPIDController controller;
    private final ElapsedTime dtTimer = new ElapsedTime();

    private double targetAngle = 0;
    private double currentAngle = 0;
    private int currentTicks = 0;
    private double lastPower = 0;

    public Turret(GamepadEx ct, Telemetry telemetry) {
        this.ct = ct;
        this.telemetry = telemetry;
        controller = new ProfiledPIDController(Kp, Ki, Kd,
                new TrapezoidProfile.Constraints(MaxVelocity, MaxAcceleration));
        controller.setTolerance(ToleranceDeg);
    }

    public void LinkComponents(HardwareMap hwMap) {
        MotorTurela = hwMap.get(DcMotorEx.class, "MotorTurela");
    }

    public void Initialize(HardwareMap hwMap) {
        LinkComponents(hwMap);

        Center = new ButtonReader(ct, GamepadKeys.Button.LEFT_STICK_BUTTON);
        ResetEncoder = new ButtonReader(ct, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        // Daca turela merge in partea gresita fata de stick, schimba FORWARD in REVERSE
        MotorTurela.setDirection(DcMotorEx.Direction.FORWARD);
        ResetToCurrent();
        MotorTurela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        dtTimer.reset();
    }

    // Pozitia curenta devine 0 grade
    private void ResetToCurrent() {
        MotorTurela.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorTurela.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        currentTicks = 0;
        currentAngle = 0;
        targetAngle = 0;
        controller.reset(0);
    }

    // ---------------- API public ----------------

    public void SetTargetAngle(double angleDegrees) {
        targetAngle = Range.clip(angleDegrees, MIN_ANGLE, MAX_ANGLE);
    }

    public double GetAngle() {
        return currentAngle;
    }

    public double GetTargetAngle() {
        return targetAngle;
    }

    public boolean IsOnTarget() {
        return Math.abs(targetAngle - currentAngle) <= ToleranceDeg;
    }

    // Unghiul turelei ca sa tinteasca un punct de pe teren, din pozitia robotului.
    // Folosesti aceleasi unitati pentru X si Y (de exemplu cm). Heading in grade, 0 = pe axa X, + = stanga.
    // Exemplu cu odometria: turret.AimAt(x, y, heading, goalX, goalY);
    public void AimAt(double robotX, double robotY, double robotHeadingDeg, double goalX, double goalY) {
        double fieldAngle = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        SetTargetAngle(NormalizeDeg(fieldAngle - robotHeadingDeg));
    }

    public void Stop() {
        MotorTurela.setPower(0);
    }

    private static double NormalizeDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    // ---------------- Bucla ----------------

    public void Controls(double dt) {
        Center.readValue();
        ResetEncoder.readValue();

        if (ResetEncoder.wasJustPressed())
            ResetToCurrent();
        if (Center.wasJustPressed())
            SetTargetAngle(0);

        double stick = ct.getLeftX();
        if (Math.abs(stick) < 0.05)
            stick = 0;
        // Stick spre dreapta = unghi mai mic (turela spre dreapta)
        if (stick != 0)
            SetTargetAngle(targetAngle - stick * ManualSpeedDegPerSec * dt);
    }

    public void Update() {
        double dt = dtTimer.seconds();
        dtTimer.reset();

        currentTicks = MotorTurela.getCurrentPosition();
        currentAngle = currentTicks / TICKS_PER_DEGREE;

        Controls(dt);

        controller.setPID(Kp, Ki, Kd);
        controller.setConstraints(new TrapezoidProfile.Constraints(MaxVelocity, MaxAcceleration));
        controller.setTolerance(ToleranceDeg);

        double power = controller.calculate(currentAngle, targetAngle);
        double error = targetAngle - currentAngle;
        if (Math.abs(error) > ToleranceDeg)
            power += Math.signum(error) * kS;
        power = Range.clip(power, -MaxPower, MaxPower);

        // Nu o lasam sa loveasca opritoarele
        if (currentAngle >= MAX_ANGLE && power > 0)
            power = 0;
        if (currentAngle <= MIN_ANGLE && power < 0)
            power = 0;

        lastPower = power;
        MotorTurela.setPower(power);
    }

    public void ShowTelemetry() {
        telemetry.addLine("=== TURELA ===");
        telemetry.addData("Unghi", "%.1f°", currentAngle);
        telemetry.addData("Tinta", "%.1f°", targetAngle);
        telemetry.addData("Eroare", "%.2f°", targetAngle - currentAngle);
        telemetry.addData("Pe tinta", IsOnTarget());
        telemetry.addData("Ticks", currentTicks);
        telemetry.addData("Putere", "%.2f", lastPower);
        if (currentAngle >= MAX_ANGLE || currentAngle <= MIN_ANGLE)
            telemetry.addLine("Turela e la limita!");
    }

    public void Run() {
        Update();
        ShowTelemetry();
    }
}
