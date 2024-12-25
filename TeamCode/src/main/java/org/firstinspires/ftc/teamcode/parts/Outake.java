package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outake {

    Servo OutDr, SRotire, GhearaOut;

    public Outake(HardwareMap hardwareMap){
        OutDr = hardwareMap.get(Servo.class, "OutakeDreapta");
        SRotire = hardwareMap.get(Servo.class, "ServoRotire");
        GhearaOut = hardwareMap.get(Servo.class, "ServoGhearaOutake");
        SRotire.setDirection(Servo.Direction.FORWARD);
    }

    public void OutakeMiscare(boolean RotireSus, boolean RotireJos, double GhearaOutake, Gamepad gamepad1){
        if(RotireSus)
            OutDr.setPosition(OutDr.getPosition() - 0.002);
        if(RotireJos)
            OutDr.setPosition(OutDr.getPosition() + 0.002);

        double PosInitial = SRotire.getPosition();
        if(gamepad1.right_stick_x > 0.005 || gamepad1.right_stick_x < -0.005)
            SRotire.setPosition(PosInitial + 0.0023 * gamepad1.right_stick_x);

        if(GhearaOutake > 0.05 && GhearaOut.getPosition() == 0)
            GhearaOut.setPosition(0.022);
        else if(GhearaOutake > 0.05 && GhearaOut.getPosition() != 0)
            GhearaOut.setPosition(0);
    }
}
