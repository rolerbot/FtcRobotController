package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class FunctiePreTunare implements Subsystem {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private boolean active = false;
    private GamepadEx ct1;
    private Telemetry telemetry;
    private double currenttargetVelocity = 1100;
    double F = 13;
    double P = 10;
    double[] stepsizes = { 10.0, 1.0, 0.1, 0.01, 0.001 };
    int stepIndex = 1;




    ButtonReader stepIncrease;
    ButtonReader Fincrease;
    ButtonReader Fdecrease;
    ButtonReader Pincrease;
    ButtonReader Pdecrease;
    ButtonReader start;
    ButtonReader stop;
    ButtonReader Vel_Up;
    ButtonReader Vel_Dn;

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

        stepIncrease = new ButtonReader(ct1, GamepadKeys.Button.B);
        Fincrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        Fdecrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        Pincrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        Pdecrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        start       = new ButtonReader(ct1, GamepadKeys.Button.A);
        stop        = new ButtonReader(ct1, GamepadKeys.Button.Y);
        Vel_Up = new ButtonReader(ct1, GamepadKeys.Button.RIGHT_BUMPER);
        Vel_Dn = new ButtonReader(ct1, GamepadKeys.Button.LEFT_BUMPER);


        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter1.setPower(0);

        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setPower(0);
    }

    public void Apas() {

        stepIncrease.readValue();
        Fincrease.readValue();
        Fdecrease.readValue();
        Pincrease.readValue();
        Pdecrease.readValue();
        start.readValue();
        stop.readValue();
        Vel_Up.readValue();
        Vel_Dn.readValue();

        if (start.wasJustPressed())
        {
            active = true;
            shooter1.setVelocity(currenttargetVelocity);
            shooter2.setVelocity(currenttargetVelocity);
        }

        if(active == true)
        {
            if (stepIncrease.wasJustPressed())
                stepIndex = (stepIndex + 1) % stepsizes.length;

            if (Fincrease.wasJustPressed())
                F += stepsizes[stepIndex];

            if (Fdecrease.wasJustPressed())
                F -= stepsizes[stepIndex];

            if (Pincrease.wasJustPressed())
                P += stepsizes[stepIndex];

            if (Pdecrease.wasJustPressed())
                P -= stepsizes[stepIndex];

            if (Vel_Up.wasJustPressed()) {
                currenttargetVelocity += 50;
                currenttargetVelocity = Math.min(5000, currenttargetVelocity);
                shooter1.setVelocity(currenttargetVelocity);
                shooter2.setVelocity(currenttargetVelocity);
            }

            if (Vel_Dn.wasJustPressed()) {
                currenttargetVelocity -= 50;
                currenttargetVelocity = Math.max(0, currenttargetVelocity);
                shooter1.setVelocity(currenttargetVelocity);
                shooter2.setVelocity(currenttargetVelocity);
            }


                 F = Math.max(0, Math.min(25, F));
                 P = Math.max(0, Math.min(25, P));
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
               shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
               shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);



            if (stop.wasJustPressed()) {

                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                active = false;
            }

            }

        telemetry.addData("Active", active);
        telemetry.addData("P", "%.3f", P);
        telemetry.addData("F", "%.3f", F);
        telemetry.addData("Target velocity (ticks/s)", "%.0f", currenttargetVelocity);
        telemetry.addData("Shooter 1 velocity (ticks/s)", "%.0f", shooter1.getVelocity());
        telemetry.addData("Shooter 2 velocity (ticks/s)", "%.0f", shooter2.getVelocity());
        telemetry.update();


    }

    public void Run()
    {
        Apas();
    }
}