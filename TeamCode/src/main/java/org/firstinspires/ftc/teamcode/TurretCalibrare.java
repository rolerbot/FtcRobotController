package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Calibrarea turelei: afla TICKS_PER_DEGREE si TICKS_AT_CENTER reale.
 *
 * Pasi:
 *  1. Impinge capul de mana la capatul din DREAPTA (acelasi loc ca la autonomie) si apasa INIT.
 *     Encoderul se reseteaza la 0 aici. Motorul e liber (FLOAT), capul se misca de mana.
 *  2. Apasa START.
 *  3. Roteste capul din mana si apasa pe gamepad 1(pe radu):
 *       A = capul priveste EXACT inainte (paralel cu robotul)
 *       B = capul priveste EXACT 90° la STANGA (perpendicular pe robot)
 *       X = capul priveste EXACT 90° la DREAPTA (daca ajunge; optional)
 *       Y = capul e la capatul maxim din STANGA
 *  4. Citeste rezultatul din telemetrie si pune valorile in TurretPinpointPIDControl.
 *  RADU DACA CITESTI ASTA SA IMI BELESI PULA TINERE ESTI UN TERMINAT
 * Dupa calibrare, encoderul ramane valid (0 = capatul din dreapta) pana la oprirea robotului.
 */
@TeleOp(name = "Calibrare Turela", group = "Tuning")
public class TurretCalibrare extends OpMode {
    private DcMotorEx MotorTurela;

    private int ticksForward = Integer.MIN_VALUE;
    private int ticksLeft90 = Integer.MIN_VALUE;
    private int ticksRight90 = Integer.MIN_VALUE;
    private int ticksMax = Integer.MIN_VALUE;

    private boolean lastA, lastB, lastX, lastY;

    @Override
    public void init() {
        MotorTurela = hardwareMap.get(DcMotorEx.class, "MotorTurela");
        // Aceeasi directie ca in TurretPinpointPIDControl, ca tickurile sa aiba acelasi semn
        MotorTurela.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorTurela.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorTurela.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorTurela.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorTurela.setPower(0);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Capul trebuie sa fie impins la capatul din DREAPTA.");
        telemetry.addLine("Encoderul a fost resetat la 0 aici. Apasa START.");
        telemetry.addData("Ticks", MotorTurela.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        MotorTurela.setPower(0);
        int ticks = MotorTurela.getCurrentPosition();

        if (gamepad1.a && !lastA) ticksForward = ticks;
        if (gamepad1.b && !lastB) ticksLeft90 = ticks;
        if (gamepad1.x && !lastX) ticksRight90 = ticks;
        if (gamepad1.y && !lastY) ticksMax = ticks;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        telemetry.addLine("Misca capul DE MANA si apasa pe gamepad 1:");
        telemetry.addLine("  A = exact inainte | B = exact 90° stanga");
        telemetry.addLine("  X = exact 90° dreapta (optional) | Y = capat maxim stanga");
        telemetry.addLine();
        telemetry.addData("Ticks acum", ticks);
        telemetry.addData("Unghi cu valorile din cod", "%.1f°",
                (ticks - TurretPinpointPIDControl.TICKS_AT_CENTER) / TurretPinpointPIDControl.TICKS_PER_DEGREE);
        telemetry.addLine();
        telemetry.addData("A inainte", Show(ticksForward));
        telemetry.addData("B 90° stanga", Show(ticksLeft90));
        telemetry.addData("X 90° dreapta", Show(ticksRight90));
        telemetry.addData("Y capat stanga", Show(ticksMax));
        telemetry.addLine();

        // ---------- rezultate ----------
        double tpd = Double.NaN;
        int count = 0;
        double sum = 0;
        if (Has(ticksForward) && Has(ticksLeft90)) {
            sum += (ticksLeft90 - ticksForward) / 90.0;
            count++;
        }
        if (Has(ticksForward) && Has(ticksRight90)) {
            sum += (ticksForward - ticksRight90) / 90.0;
            count++;
        }
        if (count > 0) tpd = sum / count;

        telemetry.addLine("=== REZULTAT (pune in TurretPinpointPIDControl) ===");
        if (Has(ticksForward)) {
            telemetry.addData("TICKS_AT_CENTER", ticksForward);
        }
        if (!Double.isNaN(tpd)) {
            telemetry.addData("TICKS_PER_DEGREE", "%.4f", tpd);
            if (tpd <= 0) {
                telemetry.addLine("ATENTIE: valoare negativa -> stanga/dreapta inversate, spune-mi!");
            }
        }
        if (Has(ticksMax)) {
            telemetry.addData("MAX_TICKS (capat stanga)", ticksMax);
            telemetry.addData("Centru din simetrie (MAX/2)", ticksMax / 2);
            telemetry.addData("Daca plaja ar fi exact 180°", "%.4f tickuri/grad", ticksMax / 180.0);
            if (!Double.isNaN(tpd) && tpd > 0) {
                telemetry.addData("Plaja reala a turelei", "%.1f°", ticksMax / tpd);
                if (Has(ticksForward)) {
                    telemetry.addData("Capat dreapta / stanga", "%.1f° / %.1f°",
                            (0 - ticksForward) / tpd, (ticksMax - ticksForward) / tpd);
                }
            }
        }
        telemetry.addLine();
        telemetry.addData("Valori actuale in cod", "%.4f tickuri/grad, centru %d",
                TurretPinpointPIDControl.TICKS_PER_DEGREE, TurretPinpointPIDControl.TICKS_AT_CENTER);
        telemetry.update();
    }

    @Override
    public void stop() {
        MotorTurela.setPower(0);
    }

    private static boolean Has(int v) {
        return v != Integer.MIN_VALUE;
    }

    private static String Show(int v) {
        return Has(v) ? String.valueOf(v) : "-";
    }
}
