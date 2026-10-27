package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

/**
 * Auto-tuning pentru PID-ul turelei (metoda "releu", Astrom-Hagglund).
 *
 * Ce face:
 *  X  = masoara kS: puterea minima la care turela incepe sa se miste (in ambele sensuri).
 *  Y  = testul releu: in jurul centrului da +P / -P dupa semnul erorii -> turela oscileaza singura.
 *       Din amplitudinea si perioada oscilatiei se calculeaza Ku (castigul critic) si Tu (perioada),
 *       apoi Kp / Kd dupa 3 reguli (agresiv / echilibrat / fara depasire).
 *  DPAD sus/jos = alege setul de valori propus.
 *  A  = test treapta (+-30 grade) cu valorile ALESE   -> depasire, timp de stabilizare.
 *  B  = test treapta cu valorile ACTUALE din TurretMotionCompensatedPID (pentru comparatie).
 *  RB = aplica valorile alese in TurretMotionCompensatedPID si TurretBallisticShotPID
 *       (doar pana la restartul aplicatiei: dupa ce sunt bune, SCRIETI-LE IN COD).
 *
 * INAINTE DE INIT: turela impinsa de mana la opritorul din DREAPTA (acolo encoderul devine 0),
 * exact ca la autonomie. Robotul pe loc, nimeni langa turela.
 *
 * Unitati: unghi in grade, putere -1..1, timp in secunde (Kp = putere/grad, Kd = putere/(grad/s)).
 */
@Configurable
@TeleOp(name = "Turela - Auto Tuning PID", group = "Tuning")
public class TurretAutoTuner extends LinearOpMode {

    // ---- reglabile din Panels ----
    public static double RELAY_POWER = 0.20;      // puterea releului (trebuie sa fie peste kS)
    public static double RELAY_HYST_DEG = 0.5;    // histerezis, ca zgomotul encoderului sa nu comute releul
    public static int RELAY_CYCLES = 6;           // perioade masurate (primele 2 se ignora)
    public static double RELAY_TIMEOUT_S = 10.0;
    public static double KS_RAMP_PER_S = 0.05;    // cat de repede creste puterea la masurarea kS
    public static double KS_MOVE_DEG_S = 4.0;     // de la viteza asta se considera ca "s-a miscat"
    public static double STEP_DEG = 30.0;         // marimea treptei la test
    public static double STEP_TIME_S = 2.0;       // cat timp se urmareste fiecare treapta
    public static double SETTLE_BAND_DEG = 1.0;   // "stabilizat" = eroare sub asta
    public static double SAFE_LIMIT_DEG = 60.0;   // peste asta orice test se opreste
    public static double MAX_POWER = 0.6;         // ca in clasele turelei

    private DcMotorEx motor;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // rezultate
    private double kSPlus = Double.NaN, kSMinus = Double.NaN;
    private double ku = Double.NaN, tu = Double.NaN, amplitude = Double.NaN;
    private final String[] ruleNames = { "Agresiv (Ziegler-Nichols PD)", "Echilibrat", "Fara depasire" };
    private final double[][] candidates = new double[3][2]; // {Kp, Kd}
    private int selected = 1;
    private String lastStep = "-";
    private String status = "Gata. Alege un test.";

    private boolean prevX, prevY, prevA, prevB, prevUp, prevDown, prevRB;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        motor = hardwareMap.get(DcMotorEx.class, "MotorTurela");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Turela trebuie sa fie la opritorul din DREAPTA (encoder = 0).");
        telemetry.addLine("Daca nu era: STOP, impinge-o acolo, INIT din nou.");
        telemetry.addLine("Apoi START.");
        telemetry.update();
        waitForStart();

        status = "Merg la centru...";
        GoTo(0.0, TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, 3.0);
        status = "Gata. Alege un test.";

        while (opModeIsActive()) {
            boolean x = gamepad1.x, y = gamepad1.y, a = gamepad1.a, b = gamepad1.b;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, rb = gamepad1.right_bumper;

            if (x && !prevX) MeasureKS();
            if (y && !prevY) RelayTest();
            if (up && !prevUp) selected = (selected + 2) % 3;
            if (down && !prevDown) selected = (selected + 1) % 3;
            if (a && !prevA && !Double.isNaN(ku)) {
                lastStep = "ales (" + ruleNames[selected] + "): "
                        + StepTest(candidates[selected][0], candidates[selected][1], KsOrZero());
            }
            if (b && !prevB) {
                lastStep = "actual: " + StepTest(TurretMotionCompensatedPID.Kp,
                        TurretMotionCompensatedPID.Kd, TurretMotionCompensatedPID.kS);
            }
            if (rb && !prevRB && !Double.isNaN(ku)) Apply();

            prevX = x; prevY = y; prevA = a; prevB = b; prevUp = up; prevDown = down; prevRB = rb;

            HoldAt(0.0);
            ShowTelemetry();
        }
        motor.setPower(0);
    }

    // ===================== teste =====================

    /** Creste puterea incet pana turela se misca; in ambele sensuri. */
    private void MeasureKS() {
        status = "kS: masor sensul + ...";
        kSPlus = RampUntilMoves(+1);
        GoTo(0.0, TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, 2.0);
        status = "kS: masor sensul - ...";
        kSMinus = RampUntilMoves(-1);
        GoTo(0.0, TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, 2.0);
        status = "kS gata.";
    }

    private double RampUntilMoves(int dir) {
        // pornim putin in partea opusa, ca sa avem loc de miscare
        GoTo(-dir * 20.0, TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, 2.0);
        motor.setPower(0);
        sleep(300);
        ElapsedTime t = new ElapsedTime();
        double power = 0;
        while (opModeIsActive() && power < 0.5) {
            power = KS_RAMP_PER_S * t.seconds();
            motor.setPower(dir * power);
            double angle = Angle();
            double vel = motor.getVelocity() / TurretMotionCompensatedPID.TICKS_PER_DEGREE;
            if (Math.abs(vel) > KS_MOVE_DEG_S || Math.abs(angle) > SAFE_LIMIT_DEG) {
                motor.setPower(0);
                holdController = null;
                return power;
            }
            telemetry.addData("kS", "sens %s, putere %.3f, viteza %.1f°/s", dir > 0 ? "+" : "-", power, vel);
            telemetry.update();
        }
        motor.setPower(0);
        holdController = null;
        return Double.NaN;
    }

    /** Releu cu histerezis in jurul lui 0 -> oscilatie -> Ku, Tu. */
    private void RelayTest() {
        status = "Releu: merg la centru...";
        GoTo(0.0, TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, 2.0);
        double h = RELAY_POWER;
        if (!Double.isNaN(kSPlus) && !Double.isNaN(kSMinus)) {
            h = Math.max(h, 1.5 * Math.max(kSPlus, kSMinus)); // releul trebuie sa invinga frecarea
        }
        status = String.format("Releu: oscilez cu %.2f putere...", h);

        List<Double> riseTimes = new ArrayList<>();
        List<Double> amps = new ArrayList<>();
        double out = h;
        double hiPeak = -1e9, loPeak = 1e9;
        ElapsedTime t = new ElapsedTime();
        boolean aborted = false;

        while (opModeIsActive() && t.seconds() < RELAY_TIMEOUT_S && riseTimes.size() < RELAY_CYCLES + 3) {
            double angle = Angle();
            if (Math.abs(angle) > SAFE_LIMIT_DEG) { aborted = true; break; }
            double err = 0.0 - angle;
            hiPeak = Math.max(hiPeak, angle);
            loPeak = Math.min(loPeak, angle);

            if (err > RELAY_HYST_DEG && out < 0) {          // comutare - -> +
                out = h;
                riseTimes.add(t.seconds());
                if (hiPeak > -1e8 && loPeak < 1e8) amps.add((hiPeak - loPeak) / 2.0);
                hiPeak = -1e9;
                loPeak = 1e9;
            } else if (err < -RELAY_HYST_DEG && out > 0) {  // comutare + -> -
                out = -h;
            }
            motor.setPower(out);
            telemetry.addData("Releu", "unghi %.1f°, iesire %+.2f, cicluri %d", angle, out, riseTimes.size());
            telemetry.update();
        }
        motor.setPower(0);
        holdController = null;

        int skip = 2; // primele perioade nu sunt inca stabile
        if (aborted || riseTimes.size() < skip + 3 || amps.size() < skip + 2) {
            status = aborted ? "Releu OPRIT: turela a iesit din zona sigura. Scade RELAY_POWER."
                    : "Releu: nu a oscilat destul. Creste RELAY_POWER (sau masoara intai kS cu X).";
            GoTo(0.0, TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, 2.0);
            return;
        }
        double sumT = 0; int nT = 0;
        for (int i = skip + 1; i < riseTimes.size(); i++) { sumT += riseTimes.get(i) - riseTimes.get(i - 1); nT++; }
        double sumA = 0; int nA = 0;
        for (int i = skip; i < amps.size(); i++) { sumA += amps.get(i); nA++; }
        tu = sumT / nT;
        amplitude = sumA / nA;
        double aEff = Math.sqrt(Math.max(amplitude * amplitude - RELAY_HYST_DEG * RELAY_HYST_DEG, 1e-6));
        ku = 4.0 * h / (Math.PI * aEff);

        candidates[0][0] = 0.8 * ku;  candidates[0][1] = 0.1 * ku * tu;    // Z-N PD
        candidates[1][0] = 0.33 * ku; candidates[1][1] = 0.11 * ku * tu;   // "some overshoot"
        candidates[2][0] = 0.2 * ku;  candidates[2][1] = 0.066 * ku * tu;  // "no overshoot"

        status = "Releu gata. Alege setul (DPAD) si testeaza cu A.";
        GoTo(0.0, TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, 2.0);
    }

    /** Treapta 0 -> +STEP -> -STEP -> 0 cu controlerul real (profil trapezoidal), raporteaza depasirea si timpul. */
    private String StepTest(double kp, double kd, double ks) {
        status = String.format("Treapta cu Kp=%.4f Kd=%.4f kS=%.3f ...", kp, kd, ks);
        GoTo(0.0, kp, kd, 1.5);
        double[] targets = { STEP_DEG, -STEP_DEG, 0.0 };
        double worstOver = 0, worstSettle = 0;
        boolean allSettled = true;
        for (double target : targets) {
            double[] r = new double[2];
            boolean ok = GoToMeasured(target, kp, kd, ks, STEP_TIME_S, r);
            worstOver = Math.max(worstOver, r[0]);
            worstSettle = Math.max(worstSettle, r[1]);
            allSettled &= ok;
        }
        status = "Treapta gata.";
        return String.format("depasire max %.1f°, stabilizare %s", worstOver,
                allSettled ? String.format("%.2f s", worstSettle) : "NU s-a stabilizat");
    }

    private void Apply() {
        double kp = candidates[selected][0], kd = candidates[selected][1], ks = KsOrZero();
        TurretMotionCompensatedPID.Kp = kp;
        TurretMotionCompensatedPID.Kd = kd;
        TurretMotionCompensatedPID.kS = ks;
        TurretBallisticShotPID.Kp = kp;
        TurretBallisticShotPID.Kd = kd;
        TurretBallisticShotPID.kS = ks;
        status = String.format("APLICAT (pana la restart): Kp=%.4f Kd=%.4f kS=%.3f. Scrie-le in cod!", kp, kd, ks);
    }

    // ===================== control =====================

    /** Merge la unghi cu controlerul real; nu masoara nimic. */
    private void GoTo(double target, double kp, double kd, double timeS) {
        GoToMeasured(target, kp, kd, TurretMotionCompensatedPID.kS, timeS, null);
    }

    /**
     * Acelasi controler ca in TurretMotionCompensatedPID (ProfiledPIDController + kS + limita de putere).
     * r[0] = depasire (grade), r[1] = timp pana ramane in SETTLE_BAND_DEG. Returneaza true daca s-a stabilizat.
     */
    private boolean GoToMeasured(double target, double kp, double kd, double ks, double timeS, double[] r) {
        ProfiledPIDController c = new ProfiledPIDController(kp, 0, kd,
                new TrapezoidProfile.Constraints(TurretMotionCompensatedPID.maxVelocity,
                        TurretMotionCompensatedPID.maxAcceleration));
        double start = Angle();
        c.reset(start);
        double dir = Math.signum(target - start);
        double over = 0;
        double settleStart = -1;
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeS) {
            double angle = Angle();
            if (Math.abs(angle) > SAFE_LIMIT_DEG + 10) break;
            double power = c.calculate(angle, target);
            double err = target - angle;
            if (Math.abs(err) > SETTLE_BAND_DEG) power += Math.signum(err) * ks;
            power = Range.clip(power, -MAX_POWER, MAX_POWER);
            motor.setPower(power);

            if (dir != 0) over = Math.max(over, (angle - target) * dir);
            if (Math.abs(err) < SETTLE_BAND_DEG) {
                if (settleStart < 0) settleStart = t.seconds();
            } else {
                settleStart = -1;
            }
            if (r != null) {
                telemetry.addData("Treapta", "tinta %.0f°, unghi %.1f°, putere %+.2f", target, angle, power);
                telemetry.update();
            }
        }
        holdController = null; // controlerul de "tinut pe loc" porneste din nou de la pozitia curenta
        if (r != null) {
            r[0] = over;
            r[1] = settleStart < 0 ? timeS : settleStart;
        }
        return settleStart >= 0;
    }

    /** Tine turela pe loc intre teste (cu valorile actuale). */
    private ProfiledPIDController holdController;
    private void HoldAt(double target) {
        if (holdController == null) {
            holdController = new ProfiledPIDController(TurretMotionCompensatedPID.Kp, 0, TurretMotionCompensatedPID.Kd,
                    new TrapezoidProfile.Constraints(TurretMotionCompensatedPID.maxVelocity,
                            TurretMotionCompensatedPID.maxAcceleration));
            holdController.reset(Angle());
        }
        double angle = Angle();
        motor.setPower(Range.clip(holdController.calculate(angle, target), -MAX_POWER, MAX_POWER));
    }

    private double Angle() {
        return (motor.getCurrentPosition() - TurretMotionCompensatedPID.TICKS_AT_CENTER)
                / TurretMotionCompensatedPID.TICKS_PER_DEGREE;
    }

    private double KsOrZero() {
        if (Double.isNaN(kSPlus) || Double.isNaN(kSMinus)) return 0.0;
        return 0.8 * Math.min(kSPlus, kSMinus); // putin sub pragul de pornire, ca sa nu oscileze
    }

    // ===================== telemetrie =====================

    private void ShowTelemetry() {
        telemetry.addLine("=== AUTO TUNING TURELA ===");
        telemetry.addData("Stare", status);
        telemetry.addData("Unghi", "%.1f°  (ticks %d)", Angle(), motor.getCurrentPosition());
        telemetry.addLine("X = kS | Y = releu | DPAD = alege | A = test ales | B = test actual | RB = aplica");
        telemetry.addLine("");
        telemetry.addData("Actual (MotionCompensated)", "Kp=%.4f Kd=%.4f kS=%.3f",
                TurretMotionCompensatedPID.Kp, TurretMotionCompensatedPID.Kd, TurretMotionCompensatedPID.kS);
        telemetry.addData("kS masurat", "+ %.3f / - %.3f  -> propus %.3f",
                kSPlus, kSMinus, KsOrZero());
        if (!Double.isNaN(ku)) {
            telemetry.addData("Releu", "Ku=%.4f  Tu=%.3f s  amplitudine=%.2f°", ku, tu, amplitude);
            for (int i = 0; i < 3; i++) {
                telemetry.addData((i == selected ? ">> " : "   ") + ruleNames[i],
                        "Kp=%.4f Kd=%.4f", candidates[i][0], candidates[i][1]);
            }
        } else {
            telemetry.addLine("Releu: inca nerulat (Y).");
        }
        telemetry.addData("Ultimul test treapta", lastStep);
        telemetry.update();
    }
}
