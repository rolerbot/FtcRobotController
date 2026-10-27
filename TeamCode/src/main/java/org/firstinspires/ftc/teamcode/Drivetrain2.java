package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drivetrain2 - varianta de test pentru Drivetrain (aceleasi motoare, aceleasi comenzi).
 * In TeleOp se inlocuieste doar tipul si "new": Drivetrain2 drivetrain = new Drivetrain2(ct1, ct2, robotPinpoint);
 *
 * Ce face in plus:
 *  1. OPRIRE LINA: cand lasi joystick-ul, puterea scade treptat (DECEL_PER_SEC), nu instant.
 *     Accelerarea ramane instant, ca sa raspunda la fel de repede.
 *  2. BRAKE cand robotul s-a oprit de tot.
 *  3. MENTINERE POZITIE: dupa ce s-a oprit, robotul tine minte pozitia (din RobotPinpoint) si,
 *     daca e impins, se intoarce singur acolo (ca in autonomie). Orice comanda de la sofer anuleaza.
 *     Siguranta: daca eroarea creste peste HOLD_GIVEUP_IN / HOLD_GIVEUP_DEG, mentinerea se opreste
 *     (ex. impins tare, sau semne gresite) pana la urmatoarea oprire.
 *
 * Conventii interne (la fel ca in Drivetrain): drive + = inapoi, strafe + = dreapta, twist + = stanga (CCW).
 * Constantele public static se pot schimba LIVE din Panels (Configurables).
 */
@Configurable
public class Drivetrain2 implements Subsystem {
    private DcMotorEx MotorFS = null; // fata stanga
    private DcMotorEx MotorFD = null; // fata dreapta
    private DcMotorEx MotorSS = null; // spate stanga
    private DcMotorEx MotorSD = null; // spate dreapta
    private ButtonReader Viteza;
    private final GamepadEx ct1, ct2;
    private final RobotPinpoint robotPinpoint;

    double schimbator = 0.4; // ca in Drivetrain (TeleOp-ul il schimba direct)

    // ===================== OPRIRE LINA =====================
    // Cat de repede scade puterea cand lasi joystick-ul (putere pe secunda).
    // 4.0 = de la viteza maxima la 0 in 0.25 s. Mai mic = oprire mai lunga si mai lina.
    public static double DECEL_PER_SEC = 4.0;
    // true = motoarele merg liber (FLOAT) cat se opreste (aluneca mai mult, ca la Pedro);
    // false = BRAKE si cat se opreste (oprirea urmeaza exact rampa de mai sus).
    public static boolean FLOAT_CAT_SE_OPRESTE = false;
    public static double STICK_DEADBAND = 0.10;  // ca la blocarea din RobotPinpoint; sub asta joystick-ul e considerat lasat

    // ===================== MENTINERE POZITIE =====================
    public static boolean HOLD_ACTIV = true;
    public static double HOLD_DELAY_S = 0.25;          // cat sta oprit inainte sa inceapa mentinerea
    public static double HOLD_MAX_SPEED_IN_S = 2.0;    // considerat oprit sub viteza asta
    public static double HOLD_MAX_TURN_DEG_S = 5.0;
    public static double HOLD_KP = 0.06;               // putere per inch de eroare
    public static double HOLD_KD = 0.004;              // putere per inch/s (amortizare)
    public static double HOLD_KP_HEADING = 0.015;      // putere per grad de eroare
    public static double HOLD_KD_HEADING = 0.0008;     // putere per grad/s
    public static double HOLD_KS = 0.05;               // doar cu HOLD_CA_PEDRO = false: putere minima contra frecarii
    public static double HOLD_KS_TURN = 0.04;          // doar cu HOLD_CA_PEDRO = false
    // true = mentinerea foloseste EXACT PIDF-urile din pedroPathing/Constants (translational + heading),
    // cu aceleasi scalari ca follower.holdPoint() (holdPointTranslationalScaling / HeadingScaling)
    // si aceeasi conversie in puteri de roti ca Mecanum din Pedro (xVelocity / yVelocity).
    // Daca refaceti tuning-ul Pedro, se schimba automat si aici. HOLD_KP/KD/KS de mai sus nu se folosesc;
    // din constantele de aici conteaza doar deadband-urile si pragul de renuntare (GIVEUP).
    public static boolean HOLD_CA_PEDRO = true;
    // Cat de "dur" tine pozitia, ca multiplicator peste calculul Pedro (P, D si F cresc la fel):
    //  1.0 = exact follower.holdPoint() din Pedro (moale)
    //  ~2.2 = cat de tare corecteaza Pedro cand urmareste un traseu (PIDF-ul fara scalarea de hold)
    // Daca robotul tremura / oscileaza pe loc -> scadeti (1.5).
    public static double HOLD_RIGIDITATE = 2.0;
    public static double HOLD_DEADBAND_IN = 0.4;       // sub eroarea asta nu corecteaza (fara tremurat)
    public static double HOLD_DEADBAND_DEG = 1.0;
    public static double HOLD_MAX_POWER = 0.45;        // putere maxima de corectie (deplasare)
    public static double HOLD_MAX_TURN_POWER = 0.35;
    public static double HOLD_GIVEUP_IN = 48.0;        // peste asta renunta (siguranta)
    public static double HOLD_GIVEUP_DEG = 90.0;
    // ============================================================

    private enum Stare { MERS, OPRIRE, OPRIT, MENTINERE, MENTINERE_ANULATA }
    private Stare stare = Stare.OPRIT;
    // diagnostic: de ce nu (mai) mentine
    private String motivAnulare = "-";
    private int nrMentineri = 0;
    private double erMaxIn = 0, erMaxDeg = 0;
    private int lastResetCount = 0;
    private double stickX = 0, stickY = 0, stickR = 0;

    private double curDrive = 0, curStrafe = 0, curTwist = 0; // comenzile dupa rampa
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stoppedTimer = new ElapsedTime();
    private boolean floatMode = false;

    private double holdX = 0, holdY = 0, holdH = 0;
    private double holdErrIn = 0, holdErrDeg = 0;
    private final double[] speeds = new double[4];

    public Drivetrain2(GamepadEx ct1, GamepadEx ct2, RobotPinpoint robotPinpoint) {
        this.ct1 = ct1;
        this.ct2 = ct2;
        this.robotPinpoint = robotPinpoint;
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
    }

    public void Initialize(HardwareMap hwMap) {
        LinkComponents(hwMap);
        Viteza = new ButtonReader(ct1, GamepadKeys.Button.B);

        for (DcMotorEx m : new DcMotorEx[] { MotorFS, MotorFD, MotorSS, MotorSD }) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        SetFloat(false);

        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFD.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSD.setDirection(DcMotorSimple.Direction.FORWARD);

        loopTimer.reset();
        stoppedTimer.reset();
    }

    public void Run() {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        Viteza.readValue();
        if (Viteza.wasJustPressed()) {
            schimbator = 1.4 - schimbator;
        }

        // comenzile soferului (exact ca in Drivetrain)
        stickX = ct1.getLeftX();
        stickY = ct1.getLeftY();
        stickR = ct1.getRightX();
        double drive = -ct1.getLeftY() * schimbator;
        double strafe = ct1.getLeftX() * schimbator;
        double twist = -ct1.getRightX() * schimbator; // joystick dreapta: dreapta = rotire spre dreapta
        if (Math.abs(drive) < STICK_DEADBAND * schimbator) drive = 0;
        if (Math.abs(strafe) < STICK_DEADBAND * schimbator) strafe = 0;
        if (Math.abs(twist) < STICK_DEADBAND * schimbator) twist = 0;
        boolean driverActive = drive != 0 || strafe != 0 || twist != 0;

        // rampa: accelerare instant, franare limitata
        double maxStep = DECEL_PER_SEC * dt;
        curDrive = Ramp(curDrive, drive, maxStep);
        curStrafe = Ramp(curStrafe, strafe, maxStep);
        curTwist = Ramp(curTwist, twist, maxStep);
        boolean rampDone = curDrive == 0 && curStrafe == 0 && curTwist == 0;

        boolean robotStill = robotPinpoint == null || (
                Math.hypot(robotPinpoint.GetVelX(), robotPinpoint.GetVelY()) < HOLD_MAX_SPEED_IN_S
                && Math.abs(Math.toDegrees(robotPinpoint.GetHeadingVelocityRad())) < HOLD_MAX_TURN_DEG_S);

        // ---------- stari ----------
        if (driverActive) {
            stare = Stare.MERS;
            stoppedTimer.reset();
        } else if (!rampDone) {
            stare = Stare.OPRIRE;
            stoppedTimer.reset();
        } else if (stare == Stare.MERS || stare == Stare.OPRIRE) {
            stare = Stare.OPRIT;
            stoppedTimer.reset();
        }

        // OPRIT sau ANULATA (dupa o impingere mare): cand robotul sta din nou, tine pozitia noua
        if (stare == Stare.OPRIT || stare == Stare.MENTINERE_ANULATA) {
            if (!robotStill) {
                stoppedTimer.reset();
            } else if (HOLD_ACTIV && robotPinpoint != null && stoppedTimer.seconds() >= HOLD_DELAY_S) {
                nrMentineri++;
                erMaxIn = 0;
                erMaxDeg = 0;
                lastResetCount = robotPinpoint.GetResetCount();
                holdX = robotPinpoint.GetX();
                holdY = robotPinpoint.GetY();
                holdH = robotPinpoint.GetHeading();
                stare = Stare.MENTINERE;
            }
        }
        if (stare == Stare.MENTINERE && !HOLD_ACTIV) {
            stare = Stare.OPRIT;
        }
        // reset de pozitie (bumper) in timpul mentinerii: punctul vechi nu mai e valid
        if (stare == Stare.MENTINERE && robotPinpoint.GetResetCount() != lastResetCount) {
            stare = Stare.MENTINERE_ANULATA;
            motivAnulare = "reset Pinpoint";
            stoppedTimer.reset();
        }

        // FLOAT doar cat se opreste (daca e cerut), altfel BRAKE
        SetFloat(FLOAT_CAT_SE_OPRESTE && stare == Stare.OPRIRE);

        double outDrive = curDrive, outStrafe = curStrafe, outTwist = curTwist;
        if (stare == Stare.MENTINERE) {
            double[] c = HoldCorrection();
            if (c == null) {
                stare = Stare.MENTINERE_ANULATA; // eroare prea mare -> renunta; cand sta, tine pozitia noua
                motivAnulare = String.format("eroare prea mare: %.1f\"  %.0f°", holdErrIn, holdErrDeg);
                stoppedTimer.reset();
            } else {
                outDrive = c[0];
                outStrafe = c[1];
                outTwist = c[2];
            }
        }

        speeds[0] = (outDrive - outStrafe + outTwist); // FS
        speeds[1] = (outDrive + outStrafe - outTwist); // FD
        speeds[2] = (outDrive + outStrafe + outTwist); // SS
        speeds[3] = (outDrive - outStrafe - outTwist); // SD

        double max = 1.0;
        for (double s : speeds) max = Math.max(max, Math.abs(s));
        MotorFS.setPower(speeds[0] / max);
        MotorFD.setPower(speeds[1] / max);
        MotorSS.setPower(speeds[2] / max);
        MotorSD.setPower(speeds[3] / max);
    }

    /** Apropie valoarea curenta de tinta: instant daca accelereaza, limitat daca franeaza / schimba sensul. */
    private static double Ramp(double current, double target, double maxStep) {
        boolean slowingDown = target == 0 || Math.signum(target) != Math.signum(current)
                || Math.abs(target) < Math.abs(current);
        if (!slowingDown || current == 0) {
            return target;
        }
        double toward = Math.signum(target) == Math.signum(current) ? target : 0.0;
        double diff = toward - current;
        if (Math.abs(diff) <= maxStep) return toward;
        return current + Math.signum(diff) * maxStep;
    }

    /**
     * Corectia pentru mentinerea pozitiei, in conventia interna {drive, strafe, twist}.
     * null = eroarea e prea mare -> se renunta (siguranta).
     */
    private double[] HoldCorrection() {
        if (HOLD_CA_PEDRO) return HoldCorrectionPedro();
        double x = robotPinpoint.GetX();
        double y = robotPinpoint.GetY();
        double hDeg = robotPinpoint.GetHeading();
        double h = Math.toRadians(hDeg);

        double ex = holdX - x;
        double ey = holdY - y;
        holdErrIn = Math.hypot(ex, ey);
        holdErrDeg = NormalizeDeg(holdH - hDeg);
        erMaxIn = Math.max(erMaxIn, holdErrIn);
        erMaxDeg = Math.max(erMaxDeg, Math.abs(holdErrDeg));

        if (holdErrIn > HOLD_GIVEUP_IN || Math.abs(holdErrDeg) > HOLD_GIVEUP_DEG) {
            return null;
        }

        // eroare si viteza in sistemul robotului (inainte, stanga)
        double cos = Math.cos(h), sin = Math.sin(h);
        double errFwd = ex * cos + ey * sin;
        double errLeft = -ex * sin + ey * cos;
        double vx = robotPinpoint.GetVelX(), vy = robotPinpoint.GetVelY();
        double vFwd = vx * cos + vy * sin;
        double vLeft = -vx * sin + vy * cos;
        double omegaDeg = Math.toDegrees(robotPinpoint.GetHeadingVelocityRad());

        double fwd = 0, left = 0, turn = 0;
        if (holdErrIn > HOLD_DEADBAND_IN) {
            fwd = HOLD_KP * errFwd - HOLD_KD * vFwd;
            left = HOLD_KP * errLeft - HOLD_KD * vLeft;
            // kS pe directia erorii: fara el, la 1-3" puterea e sub frecare si robotul nu se misca
            fwd += HOLD_KS * errFwd / holdErrIn;
            left += HOLD_KS * errLeft / holdErrIn;
            double mag = Math.hypot(fwd, left);
            if (mag > HOLD_MAX_POWER) {
                fwd *= HOLD_MAX_POWER / mag;
                left *= HOLD_MAX_POWER / mag;
            }
        }
        if (Math.abs(holdErrDeg) > HOLD_DEADBAND_DEG) {
            turn = Range.clip(HOLD_KP_HEADING * holdErrDeg - HOLD_KD_HEADING * omegaDeg
                            + Math.signum(holdErrDeg) * HOLD_KS_TURN,
                    -HOLD_MAX_TURN_POWER, HOLD_MAX_TURN_POWER);
        }

        // conventia interna: drive + = inapoi, strafe + = dreapta, twist + = stanga (CCW)
        return new double[] { -fwd, -left, turn };
    }

    /**
     * Ca follower.holdPoint() din Pedro 2.x (VectorCalculator + Mecanum.calculateDrive):
     *  - translational: |putere| = (P*e + D*de/dt + F) * holdPointTranslationalScaling, pe directia erorii
     *  - heading:       putere   = (P*e + D*de/dt + F*sens) * holdPointHeadingScaling   (e in radiani)
     *  - roti: Pedro inmulteste vectorii cu 2 si ii descompune pe vectorii rotilor (unghi din
     *    xVelocity/yVelocity) -> putere roata = inainte/cos(t) +- lateral/sin(t) +- rotire/cos(t)
     * de/dt e luat din viteza Pinpoint (aceeasi valoare, fara zgomotul derivatei numerice).
     */
    private double[] HoldCorrectionPedro() {
        double x = robotPinpoint.GetX();
        double y = robotPinpoint.GetY();
        double hDeg = robotPinpoint.GetHeading();
        double h = Math.toRadians(hDeg);

        double ex = holdX - x;
        double ey = holdY - y;
        holdErrIn = Math.hypot(ex, ey);
        holdErrDeg = NormalizeDeg(holdH - hDeg);
        erMaxIn = Math.max(erMaxIn, holdErrIn);
        erMaxDeg = Math.max(erMaxDeg, Math.abs(holdErrDeg));

        if (holdErrIn > HOLD_GIVEUP_IN || Math.abs(holdErrDeg) > HOLD_GIVEUP_DEG) {
            return null;
        }

        PIDFCoefficients tc = Constants.followerConstants.coefficientsTranslationalPIDF;
        PIDFCoefficients hc = Constants.followerConstants.coefficientsHeadingPIDF;
        double tScale = Constants.followerConstants.holdPointTranslationalScaling;
        double hScale = Constants.followerConstants.holdPointHeadingScaling;
        double maxP = Constants.driveConstants.maxPower;
        double t = Math.atan2(Constants.driveConstants.yVelocity, Constants.driveConstants.xVelocity);
        double cosT = Math.cos(t), sinT = Math.sin(t);

        double vx = robotPinpoint.GetVelX(), vy = robotPinpoint.GetVelY();
        double cos = Math.cos(h), sin = Math.sin(h);

        double fwd = 0, left = 0;
        if (holdErrIn > HOLD_DEADBAND_IN) {
            double ux = ex / holdErrIn, uy = ey / holdErrIn;   // directia spre punctul de mentinere
            double dErr = -(vx * ux + vy * uy);                 // d|e|/dt
            double mag = (tc.P * holdErrIn + tc.D * dErr + tc.F) * tScale * HOLD_RIGIDITATE;
            mag = Range.clip(mag, 0, maxP);
            double fx = ux * mag, fy = uy * mag;                // pe teren
            fwd = fx * cos + fy * sin;                          // in sistemul robotului
            left = -fx * sin + fy * cos;
        }

        double turn = 0;
        if (Math.abs(holdErrDeg) > HOLD_DEADBAND_DEG) {
            double e = Math.toRadians(holdErrDeg);
            double dErr = -robotPinpoint.GetHeadingVelocityRad();
            double sens = Math.signum(e);
            turn = (hc.P * e + hc.D * dErr + hc.F * sens) * hScale * HOLD_RIGIDITATE;
            turn = Range.clip(turn, -maxP, maxP);
        }

        // descompunerea pe roti ca in Pedro
        double wFwd = fwd / cosT, wLeft = left / sinT, wTurn = turn / cosT;
        // conventia interna: drive + = inapoi, strafe + = dreapta, twist + = stanga (CCW)
        return new double[] { -wFwd, -wLeft, wTurn };
    }

    private boolean zeroPowerSet = false;

    /** Seteaza FLOAT / BRAKE doar cand se schimba (fiecare setare e o comanda catre hub). */
    private void SetFloat(boolean wantFloat) {
        if (MotorFS == null || (zeroPowerSet && wantFloat == floatMode)) return;
        DcMotor.ZeroPowerBehavior b = wantFloat ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE;
        MotorFS.setZeroPowerBehavior(b);
        MotorFD.setZeroPowerBehavior(b);
        MotorSS.setZeroPowerBehavior(b);
        MotorSD.setZeroPowerBehavior(b);
        floatMode = wantFloat;
        zeroPowerSet = true;
    }

    private static double NormalizeDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    public void UpdateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("=== DRIVETRAIN2 ===");
        telemetry.addData("  Stare", stare);
        telemetry.addData("  Joystick (stg X/Y, dr X)", "%.2f / %.2f / %.2f  (deadband %.2f)",
                stickX, stickY, stickR, STICK_DEADBAND);
        if (stare == Stare.MENTINERE || stare == Stare.MENTINERE_ANULATA) {
            telemetry.addData("  Eroare mentinere", "%.2f\"  %.1f°", holdErrIn, holdErrDeg);
        }
        telemetry.addData("  Eroare maxima (impingere)", "%.1f\"  %.0f°", erMaxIn, erMaxDeg);
        telemetry.addData("  Mentineri pornite", nrMentineri);
        telemetry.addData("  Ultima anulare", motivAnulare);
    }
}
