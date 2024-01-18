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

@Autonomous(name="Autonomie bazata albastru coltul lung", group="Linear Opmode")
public class HautonomieAlbastruLung2024 extends GlobalScope2024
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
            //sleep(12000);
            int position = MotorFS.getCurrentPosition();
            telemetry.addData("Pozitita initiala", position);
            telemetry.update();
            position += 100;
            MotorFS.setTargetPosition(MotorFS.getCurrentPosition() + 2200);
            MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + 2200);
            MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + 2200);
            MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + 2200);
            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setPower(0.3);
            MotorFD.setPower(0.3);
            MotorSS.setPower(0.3);
            MotorSD.setPower(0.3);
            telemetry.addData("Pozitita finala", position);
            telemetry.update();
            telemetry.addData("Pozitita Curenta", MotorFS.getCurrentPosition());
            telemetry.update();
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

            MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - 5000);
            MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + 5000);
            MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + 5000);
            MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - 5000);

            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setPower(0.3);
            MotorFD.setPower(0.3);
            MotorSS.setPower(0.3);
            MotorSD.setPower(0.3);
            telemetry.addData("Pozitita finala", position);
            telemetry.update();
            telemetry.addData("Pozitita Curenta", MotorFS.getCurrentPosition());
            telemetry.update();
            while ((MotorFS.isBusy() && MotorFD.isBusy() && MotorSD.isBusy() && MotorSS.isBusy()) && !isStopRequested())
            {
                telemetry.addData("Pozitita", MotorFS.getCurrentPosition());
                telemetry.update();
            }

            MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - 650);
            MotorFD.setTargetPosition(MotorFD.getCurrentPosition() - 650);
            MotorSS.setTargetPosition(MotorSS.getCurrentPosition() - 650);
            MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - 650);

            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setPower(0.3);
            MotorFD.setPower(0.3);
            MotorSS.setPower(0.3);
            MotorSD.setPower(0.3);
            telemetry.addData("Pozitita finala", position);
            telemetry.update();
            telemetry.addData("Pozitita Curenta", MotorFS.getCurrentPosition());
            telemetry.update();
            while ((MotorFS.isBusy() && MotorFD.isBusy() && MotorSD.isBusy() && MotorSS.isBusy()) && !isStopRequested())
            {
                telemetry.addData("Pozitita", MotorFS.getCurrentPosition());
                telemetry.update();
            }

            while (opModeIsActive());
        }
    }
}
