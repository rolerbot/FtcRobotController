package org.firstinspires.ftc.teamcode.parts;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PrindereAuto {

    Servo OutakeSt, OutakeDr, SGhearaOut, SGhearaIn, BazaDr, BazaSt, InSt, InDr;
    double PozInSt[] = {0.088, 0.168, 0.649 ,1};
    double PozInDr[] = {0.0905, 0.1705, 0.6505 ,1};
    double PozOutDr[] = {0.5717, 0.4461, 0.375, 0.335, 0.2656};
    double PozOutSt[] = {0.4685, 0.3405, 0.3405, 0.3405, 0.2183};
    int pozin = 2, pozOut = 0, count = 0;

    public PrindereAuto(HardwareMap hardwareMap){
        OutakeDr = hardwareMap.get(Servo.class, "OutakeDreapta");
        OutakeSt = hardwareMap.get(Servo.class, "OutakeStanga");
        SGhearaIn = hardwareMap.get(Servo.class, "ServoGhearaIntake");
        SGhearaOut = hardwareMap.get(Servo.class, "ServoGhearaOutake");
        BazaDr = hardwareMap.get(Servo.class, "BazaDreapta");
        BazaSt = hardwareMap.get(Servo.class, "BazaStanga");
        InDr =  hardwareMap.get(Servo.class, "IntakeDreapta");
        InSt = hardwareMap.get(Servo.class, "IntakeStanga");

        Servo[] m = {OutakeDr, SGhearaIn, BazaDr, InSt};
        for(Servo servo : m)
            servo.setDirection(Servo.Direction.FORWARD);

        Servo[] n = {OutakeSt, SGhearaOut, BazaSt, InDr};
        for(Servo servo : n)
            servo.setDirection(Servo.Direction.REVERSE);

        BazaDr.setPosition(0.04);
        BazaSt.setPosition(0.08);
        InSt.setPosition(0.649);
        InDr.setPosition(0.6505);
        OutakeSt.setPosition(0.4685);
        OutakeDr.setPosition(0.5717);
        SGhearaIn.setPosition(0);
        SGhearaOut.setPosition(0);
    }

    public void ActiuneAuto(boolean OutakeSus, boolean OutakeJos, boolean IntakeSus, boolean IntakeJos){

        if(OutakeJos && pozOut > 0)
        {
            pozOut--;
            OutakeSt.setPosition(PozOutSt[pozOut]);
            OutakeDr.setPosition(PozOutDr[pozOut]);
            if(pozOut < 2) SGhearaOut.setPosition(0);/// 0.022
            else SGhearaOut.setPosition(0.022);
            pozin = 1;
        }
        if(OutakeSus && pozOut < 4)
        {
            pozOut++;
            OutakeSt.setPosition(0.48);
            if(pozOut < 4) SGhearaOut.setPosition(0.022);
            else SGhearaOut.setPosition(0);//Deschis
            OutakeSt.setPosition(PozOutSt[pozOut]);
            OutakeDr.setPosition(PozOutDr[pozOut]);//0.3405, 0.3517
        }

        if(IntakeSus && pozin < 3){
            pozin++;
            if(pozin > 1){
                OutakeSt.setPosition(PozOutSt[1]);
                OutakeDr.setPosition(PozOutDr[1]);
            }
            else SGhearaIn.setPosition(0);

            if(pozin == 1 && count == 0){
                BazaDr.setPosition(0.32);
                BazaSt.setPosition(0.36);
            }
            if(pozin == 3){
                BazaDr.setPosition(0.04);
                BazaSt.setPosition(0.08);
            }
        }
        if(IntakeJos && pozin > 0){
            pozin--;
            if(pozin == 0){
                OutakeSt.setPosition(PozOutSt[1]);
                OutakeDr.setPosition(PozOutDr[1]);
                SGhearaIn.setPosition(0.022);
            }
            else if(pozin == 2 || pozin == 1) SGhearaIn.setPosition(0.022);
            else SGhearaIn.setPosition(0);

            if(pozin == 1 && count == 0){
                BazaDr.setPosition(0.32);
                BazaSt.setPosition(0.36);
            }
            if(pozin == 3){
                BazaDr.setPosition(0.04);
                BazaSt.setPosition(0.08);
            }
        }
        InSt.setPosition(PozInSt[pozin]);
        InDr.setPosition(PozInDr[pozin]);
    }

    public void BazaExt(boolean a, boolean y){
        if(y) count = 0;
        if(a) count = 1;
    }
}
