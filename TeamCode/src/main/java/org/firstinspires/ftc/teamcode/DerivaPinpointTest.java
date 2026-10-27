package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Test de deriva a Pinpoint-ului din cauza vibratiilor.
 * NU e OpMode (e abstract) - se ruleaza prin cele 4 clase mici: DerivaPinpointMic / Mediu / Mare / Referinta.
 *
 * Ce face:
 *  1. La START, cu toate motoarele oprite: recalibreaza IMU-ul Pinpoint (robotul trebuie sa stea nemiscat).
 *  2. Porneste shooter-ul la viteza aleasa, intake-ul la putere maxima si transferul (ca in meci, fara mingi).
 *  3. Asteapta sa ajunga flywheel-ul la turatie, apoi pune pozitia in Pinpoint la (72, 72, 90°).
 *  4. Merge continuu. La fiecare 60 s noteaza cat s-au schimbat heading-ul si X/Y in minutul respectiv.
 *     Telemetria arata toate minutele, media si cat de mult difera intre ele.
 *
 * Pinpoint-ul e citit DIRECT (fara blocarea de pozitie din RobotPinpoint), ca sa se vada deriva reala.
 * Robotul trebuie sa stea pe loc tot testul. Nu atingeti robotul si masa pe care sta.
 */
public abstract class DerivaPinpointTest extends LinearOpMode {

    // pozitia "falsa" de la care masuram diferentele
    protected static final double START_X = 72.0;
    protected static final double START_Y = 72.0;
    protected static final double START_H = 90.0;

    protected static final double SPINUP_S = 3.0;      // cat asteptam sa ajunga flywheel-ul la turatie
    protected static final double MINUTE_S = 60.0;     // lungimea unei masuratori
    protected static final double TRANSFER_VELOCITY = 2800;

    /** Viteza flywheel-ului (tick/s). 0 = shooter oprit. */
    protected abstract double ShooterVelocity();

    /** true = pornesc intake-ul si transferul. */
    protected abstract boolean RunMechanisms();

    /** Numele testului, pentru telemetrie. */
    protected abstract String TestName();

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx shooter, intake, transfer;
    private VoltageSensor voltage;

    // heading "desfasurat" (nu sare la +-180)
    private double unwrappedH = 0, lastRawH = 0;

    private static class Minute {
        double dH, dX, dY, velMin, velMax, volt;
        Minute(double dH, double dX, double dY, double velMin, double velMax, double volt) {
            this.dH = dH; this.dX = dX; this.dY = dY; this.velMin = velMin; this.velMax = velMax; this.volt = volt;
        }
    }
    private final List<Minute> minutes = new ArrayList<>();

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConstants.hardwareMapName);
        pinpoint.setOffsets(Constants.localizerConstants.forwardPodY, Constants.localizerConstants.strafePodX,
                Constants.localizerConstants.distanceUnit);
        pinpoint.setEncoderResolution(Constants.localizerConstants.encoderResolution);
        pinpoint.setEncoderDirections(Constants.localizerConstants.forwardEncoderDirection,
                Constants.localizerConstants.strafeEncoderDirection);

        // rotile pe BRAKE si oprite, ca robotul sa nu alunece
        for (String name : new String[] { "MotorFS", "MotorFD", "MotorSS", "MotorSD" }) {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
        }

        // aceleasi setari ca in Shooter.java
        shooter = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(13, 0.004, 7.0, 13));

        transfer = hardwareMap.get(DcMotorEx.class, "MotorRidicareBila");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10.0, 3.0, 0.0, 12.0));

        // ca in Intake.java
        intake = hardwareMap.get(DcMotorEx.class, "MotorIN");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltage = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("TEST DERIVA PINPOINT: " + TestName());
        telemetry.addLine("FARA mingi in robot.");
        telemetry.addLine("Robotul pe loc, nimeni nu-l atinge tot testul.");
        telemetry.addLine("START -> calibrare IMU (motoare oprite), apoi pornesc mecanismele.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // 1. calibrare IMU cu totul oprit
        telemetry.addLine("Calibrez IMU-ul... NU MISCA ROBOTUL");
        telemetry.update();
        pinpoint.recalibrateIMU();
        sleep(500);

        // 2. pornesc mecanismele
        if (ShooterVelocity() > 0) shooter.setVelocity(ShooterVelocity());
        if (RunMechanisms()) {
            intake.setPower(1.0);
            transfer.setVelocity(TRANSFER_VELOCITY);
        }

        // 3. astept turatia, apoi pun pozitia de start
        ElapsedTime spin = new ElapsedTime();
        while (opModeIsActive() && spin.seconds() < SPINUP_S) {
            telemetry.addData("Pornesc mecanismele", "%.1f / %.0f s", spin.seconds(), SPINUP_S);
            telemetry.addData("Flywheel", "%.0f / %.0f tick/s", shooter.getVelocity(), ShooterVelocity());
            telemetry.update();
        }
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_H));
        sleep(50);
        pinpoint.update();
        lastRawH = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
        unwrappedH = lastRawH;

        // 4. masuratori continue
        ElapsedTime total = new ElapsedTime();
        ElapsedTime minute = new ElapsedTime();
        double[] mStart = { pinpoint.getPosition().getX(DistanceUnit.INCH),
                pinpoint.getPosition().getY(DistanceUnit.INCH), unwrappedH };
        double velMin = 1e9, velMax = -1e9;
        double h15 = Double.NaN, h30 = Double.NaN, h45 = Double.NaN;

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D p = pinpoint.getPosition();
            double x = p.getX(DistanceUnit.INCH), y = p.getY(DistanceUnit.INCH);
            double raw = p.getHeading(AngleUnit.DEGREES);
            unwrappedH += Normalize(raw - lastRawH);
            lastRawH = raw;

            double vel = shooter.getVelocity();
            velMin = Math.min(velMin, vel);
            velMax = Math.max(velMax, vel);

            double tm = minute.seconds();
            double dHnow = unwrappedH - mStart[2];
            if (Double.isNaN(h15) && tm >= 15) h15 = dHnow;
            if (Double.isNaN(h30) && tm >= 30) h30 = dHnow;
            if (Double.isNaN(h45) && tm >= 45) h45 = dHnow;

            if (tm >= MINUTE_S) {
                minutes.add(new Minute(dHnow, x - mStart[0], y - mStart[1], velMin, velMax, voltage.getVoltage()));
                mStart[0] = x; mStart[1] = y; mStart[2] = unwrappedH;
                minute.reset();
                velMin = 1e9; velMax = -1e9;
                h15 = h30 = h45 = Double.NaN;
            }

            // ---------- telemetrie ----------
            telemetry.addLine("=== DERIVA PINPOINT: " + TestName() + " ===");
            telemetry.addData("Timp total", "%.0f s  (minutul %d, %.0f s)", total.seconds(), minutes.size() + 1, tm);
            telemetry.addData("Flywheel", "%.0f tick/s (tinta %.0f)", vel, ShooterVelocity());
            telemetry.addData("Baterie", "%.2f V", voltage.getVoltage());
            telemetry.addData("Pozitie acum", "X %.2f  Y %.2f  H %.2f°", x, y, unwrappedH);
            telemetry.addData("Fata de start (72, 72, 90°)", "dX %+.2f\"  dY %+.2f\"  dH %+.2f°",
                    x - START_X, y - START_Y, unwrappedH - START_H);
            telemetry.addData("Minutul curent dH", "15s %s | 30s %s | 45s %s | acum %+.2f°",
                    Fmt(h15), Fmt(h30), Fmt(h45), dHnow);
            telemetry.addLine("");
            telemetry.addLine("--- Rezultate pe minut ---");
            for (int i = 0; i < minutes.size(); i++) {
                Minute m = minutes.get(i);
                telemetry.addData("Minutul " + (i + 1), "dH %+.2f°  dX %+.2f\"  dY %+.2f\"  | fly %.0f-%.0f  %.1fV",
                        m.dH, m.dX, m.dY, m.velMin, m.velMax, m.volt);
            }
            if (!minutes.isEmpty()) {
                double sum = 0, lo = 1e9, hi = -1e9;
                for (Minute m : minutes) { sum += m.dH; lo = Math.min(lo, m.dH); hi = Math.max(hi, m.dH); }
                telemetry.addData("MEDIE heading", "%+.2f °/min  (intre %+.2f si %+.2f)", sum / minutes.size(), lo, hi);
            } else {
                telemetry.addLine("Primul rezultat dupa 60 s.");
            }
            telemetry.update();
        }

        shooter.setVelocity(0);
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setVelocity(0);
        transfer.setPower(0);
    }

    private static double Normalize(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private static String Fmt(double v) {
        return Double.isNaN(v) ? "-" : String.format("%+.2f°", v);
    }
}
