package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaPoseMatrix;

public abstract class GlobalScope2023 extends LinearOpMode
{
    public class Controller
    {
        //public double x, y, a, b,
        //dpad_up,dpad_down,dpad_left,dpad_right,
        //lstick_x,
        //rstick_x, rstick_y,
        //rbumper, lbumper,
        //rtrigger, ltrigger;
        public double lstick_y;
        public double bumpers;
    }

    int bubuiala = 36; /// 1 cm = bubuiala * 1 Puiu UM (unitate de masura)
    public DcMotor MotorFS = null; /// Fata stanga
    public DcMotor MotorFD = null; /// Fata dreapta
    public DcMotor MotorSS = null; /// Spate stanga
    public DcMotor MotorSD = null; /// Spate dreapta
    public DcMotor MotorL1S = null; /// Motor Level 1 Stanga (de pe primul nivel)
    public DcMotor MotorL1D = null; /// Motor Level 1 Dreapta (de pe primul nivel)
    public DcMotor MotorL2D = null; /// Motor Level 2 Dreapta (de pe primul nivel)
    public DcMotor MotorL2S = null; /// Motor Level 2 Stanga (de pe primul nivel)

    void InitMotorsAuto()
    {
        MotorFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void StopMotorsRoti()
    {
        MotorFS.setPower(0);
        MotorFD.setPower(0);
        MotorSS.setPower(0);
        MotorSD.setPower(0);
    }

    void LinkComponents()
    {
        MotorFS = hardwareMap.get(DcMotor.class,"MotorFS");
        MotorFD = hardwareMap.get(DcMotor.class,"MotorFD");
        MotorSS = hardwareMap.get(DcMotor.class,"MotorSS");
        MotorSD = hardwareMap.get(DcMotor.class,"MotorSD");
        System.gc();// rugam frumos miraculosul java garbage collector sa stearga variabilele,
        // pentru ca nu le mai folosim
    }

    void Initialise()
    {
        LinkComponents();
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Init_Motors_Auto()
    {
        MotorFS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
