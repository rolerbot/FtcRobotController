package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FunctiePreTunare implements Subsystem {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private static final double TICKS_PER_REV = 28;   // goBILDA 6000 RPM motor
    private double step1 = 0.1, step2 = 0.01;
    private boolean active = false;
    private double power = 0;
    private GamepadEx ct1;
    private Telemetry telemetry;

    ButtonReader step1_plus;
    ButtonReader step1_minus;
    ButtonReader step2_plus;
    ButtonReader step2_minus;
    ButtonReader start;
    ButtonReader stop;

    public FunctiePreTunare(GamepadEx ct, Telemetry telemetry)
    {
        this.ct1 = ct;
        this.telemetry = telemetry;
    }

    public void LinkComponents(HardwareMap hwMap)
    {
        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hwMap.get(DcMotorEx.class, "shooter2");
    }

    public void Initialize(HardwareMap hwMap) {
        LinkComponents(hwMap);
        step1_plus  = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        step1_minus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        step2_plus  = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        step2_minus = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        start       = new ButtonReader(ct1, GamepadKeys.Button.A);
        stop        = new ButtonReader(ct1, GamepadKeys.Button.B);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setPower(0);

        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setPower(0);
    }

    public void Apas() {
        step1_plus.readValue();
        step1_minus.readValue();
        step2_plus.readValue();
        step2_minus.readValue();
        start.readValue();
        stop.readValue();

        if (start.wasJustPressed()) {
            power = 0.1;                 // power now matches what the motors get
            shooter1.setPower(power);
            shooter2.setPower(power);
            active = true;
        }

        if (active) {
            if (step1_plus.wasJustPressed())       power += step1;
            else if (step1_minus.wasJustPressed()) power -= step1;
            else if (step2_plus.wasJustPressed())  power += step2;
            else if (step2_minus.wasJustPressed()) power -= step2;

            power = Math.min(1.0, Math.max(0.0, power));   // clamp AFTER the buttons
            shooter1.setPower(power);
            shooter2.setPower(power);

            if (stop.wasJustPressed()) {
                power = 0;
                shooter1.setPower(0);
                shooter2.setPower(0);
                active = false;
            }
        }

        double vel1 = shooter1.getVelocity();   // ticks/sec
        double vel2 = shooter2.getVelocity();

        telemetry.addData("Active", active);
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Shooter1 ticks/s", "%.0f", vel1);
        telemetry.addData("Shooter2 ticks/s", "%.0f", vel2);
        telemetry.addData("Shooter1 RPM", "%.0f", vel1 * 60 / TICKS_PER_REV);
        telemetry.addData("Shooter2 RPM", "%.0f", vel2 * 60 / TICKS_PER_REV);
        telemetry.addData("F estimat", "%.2f", vel1 > 100 ? 32767 * power / vel1 : 0);
        telemetry.update();
    }

    public void Run()
    {
        Apas();
    }
}