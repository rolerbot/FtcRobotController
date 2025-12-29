package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;


@TeleOp
public class ShooterTuning extends OpMode
{

    private DcMotor shooterMotor1, shooterMotor2;
    private double highVelocity = 1600;
    private double lowVelocity = 1000;
    private double currenttargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepsizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;
    ButtonReader switchCurrentVelocity, stepIncrease, Fincrease, Fdescrease, Pincrease, Pdecrease;
    GamepadEx ct1;

    @Override
    public void init()
    {
        ct1 = new GamepadEx(gamepad1);
        shooterMotor1  = hardwareMap.get(DcMotorEx.class, "MotorAruncare1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "MotorAruncare2");
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new  PIDFCoefficients(P,0, 0, F);
        ((DcMotorEx) shooterMotor1).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        ((DcMotorEx) shooterMotor2).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        switchCurrentVelocity = new ButtonReader(ct1, GamepadKeys.Button.A);
        stepIncrease = new  ButtonReader(ct1, GamepadKeys.Button.B);
        Fincrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        Fdescrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        Pincrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        Pdecrease = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        telemetry.addLine("Initialized");
    }

    @Override
    public void loop()
    {
        //get button inputs
        switchCurrentVelocity.readValue();
        stepIncrease.readValue();
        Fincrease.readValue();
        Fdescrease.readValue();
        Pincrease.readValue();
        Pdecrease.readValue();
        if(switchCurrentVelocity.wasJustPressed())
        {
            if(currenttargetVelocity == highVelocity)
                currenttargetVelocity = lowVelocity;
            else
                currenttargetVelocity = highVelocity;
        }

        if(stepIncrease.wasJustPressed())
            stepIndex = (stepIndex + 1) % stepsizes.length;

        if(Fincrease.wasJustPressed())
            F += stepsizes[stepIndex];

        if(Fdescrease.wasJustPressed())
            F -= stepsizes[stepIndex];

        if(Pincrease.wasJustPressed())
            P += stepsizes[stepIndex];

        if(Pdecrease.wasJustPressed())
            P -= stepsizes[stepIndex];

        //set new PIDF values
        PIDFCoefficients pidfCoefficients = new  PIDFCoefficients(P,0, 0, F);
        ((DcMotorEx) shooterMotor1).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        ((DcMotorEx) shooterMotor2).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        ((DcMotorEx) shooterMotor1).setVelocity(currenttargetVelocity);
        ((DcMotorEx) shooterMotor2).setVelocity(currenttargetVelocity);

        double curVel = ((DcMotorEx) shooterMotor1).getVelocity();
        double error = currenttargetVelocity - curVel;

        //telemetry
        telemetry.addData("Target Velocity: ", currenttargetVelocity);
        telemetry.addData("Current Velocity: ", curVel);
        telemetry.addData("Error: ", error);
        telemetry.addLine("PIDF Tuning------------");
        telemetry.addData("F(D-left): ", F);
        telemetry.addData("P(D-UP): ", P);
        telemetry.addData("(B-increase)Step Size: ", stepsizes[stepIndex]);
        telemetry.update();

    }
}
