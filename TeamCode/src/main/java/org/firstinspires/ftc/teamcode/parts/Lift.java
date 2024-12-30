package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor SLiderStanga;
    private DcMotor SliderDreapta;

    private enum Positions{
        RESET,
        MIDDLE,
        TOP
    };

    private Positions position;

    public Lift(HardwareMap hardwareMap){
        SLiderStanga = hardwareMap.get(DcMotor.class, "SliderS");
        SliderDreapta = hardwareMap.get(DcMotor.class, "SliderD");
        DcMotor[] m = {SLiderStanga, SliderDreapta};
        for(DcMotor motor : m){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
    }

    private void ChangePos(Positions position)
    {
        switch (position){
            case RESET:
                SLiderStanga.setTargetPosition(0);
                SliderDreapta.setTargetPosition(0);
                break;
            case MIDDLE:
                SLiderStanga.setTargetPosition(500);
                SliderDreapta.setTargetPosition(500);
                break;
            case TOP:
                SLiderStanga.setTargetPosition(1000);
                SliderDreapta.setTargetPosition(1000);
                break;
        }
    }

    public void pressedButton(boolean y){
       if(y && position == Positions.MIDDLE)
           position = Positions.TOP;
        return;
    }
    public void PressedButton(boolean a){
        if(a && position == Positions.MIDDLE)
            position = Positions.RESET;
        return;
    }
}
