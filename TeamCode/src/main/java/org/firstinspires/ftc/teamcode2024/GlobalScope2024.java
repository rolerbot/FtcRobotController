/// REGULA NR 1 - TOMA SCRIE COD PERFECT
package org.firstinspires.ftc.teamcode2024;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class GlobalScope2024 extends LinearOpMode
{
    /// GENERAL SYSTEMS

    public DcMotorEx MotorFS = null; /// Fata stanga
    public DcMotorEx MotorFD = null; /// Fata dreapta
    public DcMotorEx MotorSS = null; /// Spate stanga
    public DcMotorEx MotorSD = null; /// Spate dreapta
    public DcMotorEx mb1 = null; /// motor brat 1 control hub port 1, motor stanga
    public DcMotorEx mb2 = null; /// motor brat 2 control hub port 2, motor dreapta
    public DcMotorEx ky5 = null;
    public Servo SDrone = null; // lanseaza avion
    public Servo SCutie = null;

    void LinkComponents()
    {
        MotorFS = hardwareMap.get(DcMotorEx.class,"MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class,"MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class,"MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class,"MotorSD");
        mb1 = hardwareMap.get(DcMotorEx.class,"mb1");
        mb2 = hardwareMap.get(DcMotorEx.class,"mb2");
        ky5 = hardwareMap.get(DcMotorEx.class,"ky5");
        SDrone = hardwareMap.get(Servo.class, "SDrone");
        SCutie = hardwareMap.get(Servo.class, "SCutie");
    }

    void Initialise()
    {
        LinkComponents();
        //---------------------ROTZI---------------
        MotorFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setTargetPositionTolerance(5); /// Eroare de 10 unitati I guess idk
        MotorFD.setTargetPositionTolerance(5);
        MotorSS.setTargetPositionTolerance(5);
        MotorSD.setTargetPositionTolerance(5);
        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);

        //--------------------------BRATZ-------------
        mb1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); ///mereu sa fie pe brake
        mb2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//
        mb1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//
        mb2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//
        mb1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ky5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mb2.setDirection(DcMotorSimple.Direction.REVERSE); //vedem daca trebuie sa schibam directia la teste
        mb1.setDirection(DcMotorSimple.Direction.FORWARD);
        SCutie.setDirection(Servo.Direction.FORWARD);

        //--------------AVION-------
        SDrone.setDirection(Servo.Direction.REVERSE); /// hopefully - daca face invers schimbam pe reverse
    }

    void StopMotors()
    {
        MotorFS.setPower(0.0);
        MotorFD.setPower(0.0);
        MotorSS.setPower(0.0);
        MotorSD.setPower(0.0);
    }



    /// TELEOP

    double drive;
    double strafe;
    double twist;
    double[] speeds = new double[4];
    double schimbator = 0.4;
    double schimbator_brat = 0.2;
    boolean YEET_THE_DRONE = false; /// Devine 1 daca a fost lansata drona
    boolean stateRotire;
    boolean clesteDreapta = true, clesteStanga = true;
    double c1Poz, c2Poz;
    int poz;
    double SCutieArray[] = {0, 0.37, 0.39, 0.29};
    int BratArray[] = {0, 0, 505, 505};
    GamepadEx ct1;
    GamepadEx ct2;
    ButtonReader LeftClesteOpener, RightClesteOpener;
    ButtonReader IAMSPEED; /// cautator de viteze
    ButtonReader Launch;
    ButtonReader ARMISSPEED;
    ButtonReader SetPoz1;
    ButtonReader Hang;
    ButtonReader Fall;
    /// THIS IS NOT A JOJO REFERENCE
    ButtonReader Aerosmith;
    ButtonReader Oasis;
    // THERE ARE TWO JOJO REFERENCES

    void WeGottaMove()
    {
        IAMSPEED.readValue();
        telemetry.addData("viteza este", schimbator);
        telemetry.update();
        if(IAMSPEED.wasJustPressed())
        {
            schimbator = 1.10 - schimbator; ///deocamdata avem doar 2 viteze dar daca e nevoie de mai multe facem
            telemetry.addData("viteza este", schimbator);
            telemetry.update();
        }
        drive  = -gamepad1.left_stick_y * schimbator;//-gamepad1.left_stick_y*0.3-
        strafe = gamepad1.left_stick_x * schimbator;//gamepad1.left_stick_x*0.3b
        twist  = (gamepad1.right_trigger-gamepad1.left_trigger)/2.5 ;
        speeds[0]=(drive + strafe + twist);//FS
        speeds[1]=(drive - strafe - twist);//FD
        speeds[2]=(drive - strafe + twist);//SS
        speeds[3]=(drive + strafe - twist);//SD
        ///telemetry.addData("axa y", gamepad1.right_stick_y);
        ///telemetry.addData("axa x", gamepad1.right_stick_x);
        ///telemetry.update();
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        // apply the calculated values to the motors.
        MotorFS.setPower(speeds[0]);
        MotorFD.setPower(speeds[1]);
        MotorSS.setPower(speeds[2]);
        MotorSD.setPower(speeds[3]);
    }

    void InitMiscare() /// chestiile de trebuie sa le pui de fiecare data ca sa se miste
    {
        MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFS.setPower(0.3);
        MotorFD.setPower(0.3);
        MotorSS.setPower(0.3);
        MotorSD.setPower(0.3);
    }

    void RotateStanga() /// Numele se explica singur
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - 1100);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + 1100);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() - 1100);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + 1100);
        InitMiscare();
    }

    void RotateDreapta() /// Numele se explica singur
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() + 1100);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() - 1100);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + 1100);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - 1100);
        InitMiscare();
    }

    void MiscareStanga(int x)
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - x);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + x);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + x);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - x);
        InitMiscare();
    }
    void MiscareDreapta(int x)
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() + x);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() - x);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() - x);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + x);
        InitMiscare();
    }
    void MiscareFata(int x)
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() + x);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + x);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + x);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + x);
        InitMiscare();
    }

    void MiscareSpate(int x)
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - x);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() - x);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() - x);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - x);
        InitMiscare();
    }

    void WeGottaLift()
    {
        ARMISSPEED.readValue();
        //telemetry.addData("viteza bratului este", schimbator_brat);
        //telemetry.update();
        if(ARMISSPEED.wasJustPressed())
        {
            schimbator_brat = 0.7 - schimbator_brat; ///deocamdata avem doar 2 viteze dar daca e nevoie de mai multe facem
            telemetry.addData("viteza bratului este", schimbator);
            telemetry.update();
        }
        if (gamepad2.dpad_up)
        {
            mb1.setPower(schimbator_brat);
            mb2.setPower(schimbator_brat);
        }
        else if (gamepad2.dpad_down)
        {
            mb1.setPower(-schimbator_brat);
            mb2.setPower(-schimbator_brat);
        }
        else
        {
            mb1.setPower(0);
            mb2.setPower(0);
        }
    }

    void WeGottaLunchDrone()
    {
        Launch.readValue();
        if (Launch.wasJustPressed())
        {
            telemetry.addData("RUMBLLLLLLLLLLLIIIIINGGG",  "ITS COOOOOOOMMMMMMMIIIIIIIINNNNNNGG");
            telemetry.update();
            if (YEET_THE_DRONE == false)
            {
                double CurrPos = SDrone.getPosition();
                SDrone.setPosition(0.9); /// vedem daca 0.3 e bine, deocamdata e pus la misto
                sleep(1500);
                YEET_THE_DRONE = true;
                SDrone.setPosition(0);
            }
        }
    }

    void WeGottaDoArmMovements()
    {
        SetPoz1.readValue();
        if (SetPoz1.wasJustPressed())
        {
            poz = 1;
            telemetry.addData("am intrat", poz);
            telemetry.addData("curent", SCutie.getPosition());
            telemetry.addData("pozitia", SCutieArray[poz]);
            telemetry.update();
            SCutie.setPosition(SCutieArray[poz]);
        }

        Aerosmith.readValue();
        if (Aerosmith.wasJustPressed() && poz < 3)
        {
            poz++;
            if (poz == 3)
            {
                SCutie.setPosition(SCutieArray[poz]);
            }
            else
            {
                mb1.setTargetPosition(BratArray[poz]);
                mb2.setTargetPosition(BratArray[poz]);
                mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mb1.setPower(0.2);
                mb2.setPower(0.2);
                sleep(500);
                SCutie.setPosition(SCutieArray[poz]);
                while (mb1.isBusy() && !isStopRequested()) {
                    telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                    telemetry.update();
                }
            }
            telemetry.addData("pozitia brat", mb1.getCurrentPosition());
            telemetry.update();
        }

        Oasis.readValue();
        if (Oasis.wasJustPressed() && poz > 1)
        {
            poz--;
            if (poz == 2)
            {
                SCutie.setPosition(SCutieArray[poz]);
            }
            else
            {
                mb1.setTargetPosition(BratArray[poz]);
                mb2.setTargetPosition(BratArray[poz]);
                mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                mb1.setPower(0.2);
                mb2.setPower(0.2);
                SCutie.setPosition(SCutieArray[poz]);
                while (mb1.isBusy() && !isStopRequested()) {
                    telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                    telemetry.update();
                }
            }
        }

        if (gamepad1.dpad_up )
        {
            SCutie.setPosition(SCutie.getPosition() - 0.001);
        }


    }

    void WeGottaKillOurselves()
    {
        Hang.readValue();
        if (Hang.wasJustPressed())
        {
            ky5.setPower(0.25);
        }

        Fall.wasJustPressed();
        if (Fall.wasJustPressed())
        {
            ky5.setPower(-0.25);
        }

        ky5.setPower(0);
    }

    /**
    void FlippyAction()
    {
        if (stateRotire == false)
        {
            rot.setPosition(0.65); // invers
            stateRotire = true;
        }
        else
        {
            rot.setPosition(0.0); // prinde pixeli
            stateRotire = false;
        }
    }

    void WeGottaDoArmMovements()
    {
        Rotate.readValue();
        LeftClesteOpener.readValue();
        RightClesteOpener.readValue();
        if (Rotate.wasJustPressed())
        {
            telemetry.addData("RUMBLLLLLLLLLLLIIIIINGGG",  rot.getPosition());
            telemetry.update();
            //sleep(1500);
            //FlippyAction();
            c2.setPosition(0.3);
            c1.setPosition(0.94);
        }
        if (LeftClesteOpener.wasJustPressed())
        {
            telemetry.addData("cleste stanga", c2.getPosition());
            telemetry.update();
            if (clesteStanga) //deschide
            {
                c2.setPosition(0.3);
                clesteStanga = false;
            } // inchide
            else {
                c2.setPosition(c2Poz);
                clesteStanga = true;
            }
        }

        if (RightClesteOpener.wasJustPressed())
        {
            telemetry.addData("cleste dreapta", c1.getPosition());
            telemetry.update();
            if (clesteDreapta) //deschide
            {
                c1.setPosition(0.94);
                clesteDreapta = false;
            }
            else // inchide
            {
                c1.setPosition(c1Poz);
                clesteDreapta = true;
            }
        }

        if (gamepad2.right_bumper)
        {
            inc.setPosition(0.575);
        }
        else if (gamepad2.left_bumper)
        {
            inc.setPosition(0.63);
        }

    }
     */

}
