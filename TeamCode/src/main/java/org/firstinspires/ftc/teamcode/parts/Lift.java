package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;

    private enum Positions{
        RESET,
        MIDDLE,
        TOP
    };

    private Positions position;

    public Lift(HardwareMap hardwareMap){
        liftMotor1 = hardwareMap.get(DcMotor.class, "lm1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lm2");
        DcMotor[] m = {liftMotor1,liftMotor2};
        for(DcMotor motor : m){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void ChangePos(Positions position)
    {
        switch (position){
            case RESET:
                liftMotor1.setTargetPosition(0);
                liftMotor2.setTargetPosition(0);
                break;
            case MIDDLE:
                liftMotor1.setTargetPosition(500);
                liftMotor2.setTargetPosition(500);
                break;
            case TOP:
                liftMotor1.setTargetPosition(1000);
                liftMotor2.setTargetPosition(1000);
                break;
        }
    }

    public void pressedButton(boolean a){
       if(a && position == Positions.MIDDLE)
           position = Positions.TOP;
        return;
    }
}
