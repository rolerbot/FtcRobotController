/// REGULA NR 1 - TOMA SCRIE COD PERFECT
package org.firstinspires.ftc.teamcode2024;

import android.widget.Button;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PuiuFaceDrifturiKaLaKaufland", group="Linear Opmode")
public class TeleOp2024 extends GlobalScope2024
{

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
        Initialise();
        waitForStart();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);

        IAMSPEED = new ButtonReader(ct1, GamepadKeys.Button.B);
        Launch = new ButtonReader(ct2, GamepadKeys.Button.B);
        ARMISSPEED = new ButtonReader(ct2, GamepadKeys.Button.A);
        SetPoz1 = new ButtonReader(ct2, GamepadKeys.Button.Y);
        Aerosmith = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_BUMPER);
        Oasis = new ButtonReader(ct2, GamepadKeys.Button.LEFT_BUMPER);
        Hang = new ButtonReader(ct1, GamepadKeys.Button.DPAD_UP);
        Fall = new ButtonReader(ct1, GamepadKeys.Button.DPAD_DOWN);
        //Rotate = new ButtonReader(ct2, GamepadKeys.Button.X);
        //LeftClesteOpener = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        //RightClesteOpener = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);

        //SCutie.setDirection(Servo.Direction.REVERSE);
        //SCutie.setPosition(1.3);

        mb1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mb2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    double arm_poz = 0.5;
    SCutie.setPosition(0.31);
        while (opModeIsActive())
        {

             //* CEL MAI UTIL COD PE CARE L-AM SCRIS VREODATA
            // * NU MAI MAI (*insert F word) ATAT CU SERVOURILE
/**
            if (gamepad1.dpad_left) {
               // SCutie.setPosition(arm_poz);
                arm_poz += 0.01;
            }
            else if (gamepad1.dpad_right) {
               // SCutie.setPosition(arm_poz);
                arm_poz -= 0.01;
            }
            arm_poz = Range.clip(arm_poz, 0, 1);
            SCutie.setPosition(arm_poz);


           telemetry.addData("SCUTIE %.2f", arm_poz);
            telemetry.update();

            sleep(40);
*/
           WeGottaMove();
           WeGottaLift();
           WeGottaLunchDrone();
           WeGottaDoArmMovements();
           WeGottaKillOurselves();
        }

        /**
        ///seteaza clestele in pozitia initiala
        rot.setPosition(0.0);
        stateRotire = false;
        c1.setDirection(Servo.Direction.FORWARD);
        c2.setDirection(Servo.Direction.FORWARD);
        c1Poz = 0.8; //stanga
        c2Poz = 0.32; //dreapta
        c1.setPosition(c1Poz); //inchide
        c2.setPosition(c2Poz); //inchide
        telemetry.addData("rot = ", rot.getPosition());
        telemetry.update();
        while (opModeIsActive())
        {
            telemetry.addData("c1 = ", c1.getPosition());
            telemetry.addData("c2 = ", c2.getPosition());
            telemetry.update();
            WeGottaMove();
            WeGottaLift();
            WeGottaLunchDrone();
            WeGottaDoArmMovements();
        }
         */
    }
}
