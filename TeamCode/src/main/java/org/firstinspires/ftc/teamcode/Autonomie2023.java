package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

@Autonomous(name="Autonomie ig", group="Linear Opmode")
public class Autonomie2023 extends GlobalScope {
    public void runOpMode() {
        Initialise();
        int pozObj=0;
        unghiZero = unghiFix = Gyro.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        Init_Motors_Auto();
        MotorBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        ServoBrat.setPower(-1);
        sleep(900);
        Ridicare(2,1);
        Move(57,0.7,'F');
        sleep(100);
        telemetry.addData("ameu  ",SenzorP1.alpha());
        telemetry.addData("aista  ",SenzorP2.alpha());
        if(SenzorP1.alpha()>250 && SenzorP1.alpha()>SenzorP2.alpha()) {
            telemetry.addData("skema ", 2);
            pozObj=2;
        }
        else if(SenzorP2.alpha()>250){
            telemetry.addData("manevra ",3);
            pozObj=3;
        }
        else {
            telemetry.addData("persusta ",1);
            pozObj=1;
        }
        telemetry.update();
        /** Move(30,1,'B');
         RotateAuto(55,-1);
         Ridicare(2,-1);
         Move(20,0.6,'F');
         RotateAuto(-30,1);
         Move(8,0.6,'F');

         if(pozObj==1) {
         Ridicare(0,1);
         Move(25,0.7,'F');
         }
         else if(pozObj==2) {
         Ridicare(1,1);
         Move(20,0.7,'F');
         }
         else {
         Ridicare(2,1);
         Move(20,0.7,'F');
         MotorBrat.setPower(0.3);
         sleep(200);
         MotorBrat.setPower(0);
         }
         Release();
         sleep(100);
         if(pozObj==1)Move(40,0.7,'B');
         else Move(35,0.7,'B');
         Ridicare(0,-1);
         RotateAuto(-15, 0.8f);
         Move(13, 0.5 , 'B');
         RotateAuto(69, -0.8f); //nice
         Move(83, 0.4f, 'B');
         RotateRata(1);
         Move(5, 0.05f, 'B');
         RotateRata(1);
         ServoBrat.setPower(-1);
         Ridicare(1,1);
         Move(280, 1, 'F');
         //Move(30, 1.0, 'B');
         //RotateAuto(95, 0.3f);
         //Move(80, 0.25, 'B');
         //RotateRata(1);
         */
        /**Move(100, 1.0, 'F');
        RotateAuto(30, 1);
        RotateRata(1);
        sleep(1000);*/
        Ridicare(2, 1);
        while(opModeIsActive());
    }
}
