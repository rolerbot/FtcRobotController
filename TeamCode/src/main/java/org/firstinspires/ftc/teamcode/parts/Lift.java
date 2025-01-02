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

    private Positions position = Positions.RESET;

    public Lift(HardwareMap hardwareMap){
        SLiderStanga = hardwareMap.get(DcMotor.class, "SliderS");
        SliderDreapta = hardwareMap.get(DcMotor.class, "SliderD");
        DcMotor[] m = {SLiderStanga, SliderDreapta};
        for(DcMotor motor : m){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
    }

    private void ChangePos(Positions altceva)
    {
        switch (altceva){
            case RESET:
                SLiderStanga.setTargetPosition(0);
                SliderDreapta.setTargetPosition(0);
                SliderDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SLiderStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case MIDDLE:
                SLiderStanga.setTargetPosition(700);
                SliderDreapta.setTargetPosition(700);
                SliderDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SLiderStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case TOP:
                SLiderStanga.setTargetPosition(2400);
                SliderDreapta.setTargetPosition(2400);
                SliderDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SLiderStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
    }

    public void pressedButton(boolean y){
       if(y && position == Positions.RESET && SLiderStanga.getCurrentPosition() < 10){
            position = Positions.MIDDLE;
            ChangePos(position);
        }
       else if(y && position == Positions.MIDDLE && SLiderStanga.getCurrentPosition() > 600){
           position = Positions.TOP;
            ChangePos(position);
       }
    }

    public void PressedButton(boolean a){
        if(a){
            position = Positions.RESET;
            ChangePos(position);
        }
        //else if(a && position == Positions.TOP && SLiderStanga.getCurrentPosition() > 2300){
          //  position = Positions.MIDDLE;
        //    ChangePos(position);
       // }
    }
}
