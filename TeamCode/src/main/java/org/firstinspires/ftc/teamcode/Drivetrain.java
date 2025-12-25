package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain implements Subsystem {
    private DcMotorEx MotorFS = null;
    /// Fata stanga
    private DcMotorEx MotorFD = null;
    /// Fata dreapta
    private DcMotorEx MotorSS = null;
    /// Spate stanga
    private DcMotorEx MotorSD = null;
    private ButtonReader Viteza;
    private final GamepadEx ct1, ct2;
    double schimbator = 0.4;//Viteza 0.4
    double[] speeds = new double[4];
    double drive, strafe, twist;
    public Drivetrain(GamepadEx ct1, GamepadEx ct2) {this.ct1 = ct1; this.ct2 = ct2;}
    public void LinkComponents(HardwareMap hardwareMap)
    {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        Viteza = new ButtonReader(ct1, GamepadKeys.Button.B);

        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void Run()
    {
        Viteza.readValue();
        if (Viteza.wasJustPressed())
        {
            schimbator = 1.4 - schimbator;
            //telemetry.addData("viteza este", schimbator);
            //telemetry.update();
        }
        drive = -ct1.getLeftY()  * schimbator;
        strafe = ct1.getLeftX() * schimbator;
        twist = schimbator * (ct1.gamepad.right_trigger - ct1.gamepad.left_trigger);
        speeds[0] = (drive + strafe + twist);//FS
        speeds[1] = (drive - strafe - twist);//FD
        speeds[2] = (drive - strafe + twist);//SS
        speeds[3] = (drive + strafe - twist);//SD
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++)
        {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }
        if (max > 1)
        {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        MotorFS.setPower(speeds[0]);
        MotorFD.setPower(speeds[1]);
        MotorSS.setPower(speeds[2]);
        MotorSD.setPower(speeds[3]);
    }
}
