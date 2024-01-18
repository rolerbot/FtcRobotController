/// REGULA NR 1 - TOMA SCRIE COD PERFECT
package org.firstinspires.ftc.teamcode2024;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Autonomie bazata albastru coltul scurt", group="Linear Opmode")
public class HautonomieAlbastru2024 extends GlobalScope2024
{
    public void runOpMode()
    {
        Initialise();

        MotorFS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        {
            MotorFS.setTargetPosition(MotorFS.getCurrentPosition() + 200);
            MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + 200);
            MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + 200);
            MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + 200);
            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setPower(0.3);
            MotorFD.setPower(0.3);
            MotorSS.setPower(0.3);
            MotorSD.setPower(0.3);
            while ((MotorFS.isBusy() && MotorFD.isBusy() && MotorSD.isBusy() && MotorSS.isBusy()) && !isStopRequested())
            {
                telemetry.addData("Pozitita", MotorFS.getCurrentPosition());
                telemetry.update();
            }

            /** rotire la stanga
            MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - 1100);
            MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + 1100);
            MotorSS.setTargetPosition(MotorSS.getCurrentPosition() - 1100);
            MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + 1100);
             */

            MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - 2400);
            MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + 2400);
            MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + 2400);
            MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - 2400);

            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setPower(0.3);
            MotorFD.setPower(0.3);
            MotorSS.setPower(0.3);
            MotorSD.setPower(0.3);
            while ((MotorFS.isBusy() && MotorFD.isBusy() && MotorSD.isBusy() && MotorSS.isBusy()) && !isStopRequested())
            {
                telemetry.addData("Pozitita", MotorFS.getCurrentPosition());
                telemetry.update();
            }



            while (opModeIsActive());
        }
    }
}
