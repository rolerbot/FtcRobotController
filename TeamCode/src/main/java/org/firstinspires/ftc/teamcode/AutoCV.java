package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@Autonomous(name = "Test", group = "Autonomous")
public class AutoCV extends GlobalScope {

    public class Lift
    {
        public class LiftUp implements Action
        {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if (!initialized)
                {
                    SliderS.setPower(0.5);
                    SliderD.setPower(0.5);
                    initialized = true;
                }
                double pos = SliderS.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2400)
                    return true;
                else
                {
                    //SliderS.setPower(0);
                    //SliderD.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp()
        {
            return new LiftUp();
        }

        public class LiftDown implements Action
        {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if (!initialized)
                {
                    SliderS.setPower(-0.5);
                    SliderD.setPower(-0.5);
                    initialized = true;
                }
                double pos = SliderS.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 30)
                    return true;
                else
                {
                    SliderS.setPower(0);
                    SliderD.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown()
        {
            return new LiftDown();
        }


    }

    public class Cleste
    {
        public class CloseClaw implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                ServoGhearaOutake.setPosition(0.022);
                return false;
            }
        }
        public Action closeClaw()
        {
            return new CloseClaw();
        }

        public class OpenClaw implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                ServoGhearaOutake.setPosition(0);
                return false;
            }
        }
        public Action openClaw()
        {
            return new OpenClaw();
        }
    }

    public class BratOutake
    {
        public class Poz2 implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                OutakeStanga.setPosition(PozOutakeStanga[2]);
                OutakeDreapta.setPosition(PozOutakeDreapta[2]);
                SliderS.setPower(0.5);
                SliderD.setPower(0.5);
                if (OutakeStanga.getPosition() == PozOutakeStanga[2] && OutakeDreapta.getPosition() == PozOutakeDreapta[2])
                    return false;
                else return true;
            }
        }

        public Action doPoz2()
        {
            return new Poz2();
        }

        public class Poz0 implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                OutakeStanga.setPosition(PozOutakeStanga[0]);
                OutakeDreapta.setPosition(PozOutakeDreapta[0]);
                SliderS.setPower(0.5);
                SliderD.setPower(0.5);
                return false;
            }
        }

        public Action doPoz0()
        {
            return new Poz0();
        }
    }



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0)); //11.8, 61.7
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift();
        Cleste claw = new Cleste();
        BratOutake bratoutake = new BratOutake();
        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                /**.lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);*/
                //.strafeTo(new Vector2d(15, 18))
                //.strafeTo(new Vector2d(20.5, 0))
                .strafeTo(new Vector2d(10, 55))
                .turn(Math.toRadians(-45));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                /**.lineToYSplineHeading(33, Math.toRadians(0))
                 .waitSeconds(2)
                 .setTangent(Math.toRadians(90))
                 .lineToY(50)
                 .setTangent(Math.toRadians(0))
                 .lineToX(32)
                 .strafeTo(new Vector2d(44.5, 30))
                 .turn(Math.toRadians(180))
                 .lineToX(47.5)
                 .waitSeconds(3);*/
                //.strafeTo(new Vector2d(15, 18))
                //.strafeTo(new Vector2d(20.5, 0))
                .strafeTo(new Vector2d(10, -75));
               // .turn(Math.toRadians(-45));


        TrajectoryActionBuilder tabWait = drive.actionBuilder(initialPose)
                        .waitSeconds(4);


        waitForStart();

        if (isStopRequested()) return;

        Initialise();
        telemetry.addData("pozitia brat", SliderS.getCurrentPosition());
        telemetry.addData("pozitia brat2", SliderD.getCurrentPosition());
        telemetry.update();
        //OutakeDreapta.setPosition(PozOutakeDreapta[0]);
        //OutakeStanga.setPosition(PozOutakeStanga[0]);
        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );
        /**
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        tab.build(),
                        lift.liftUp(),
                       // tabWait.build(),
                        bratoutake.doPoz2(),
                        claw.openClaw(),
                        bratoutake.doPoz0(),
                        lift.liftDown()

                )
        );
    /**
        SliderS.setTargetPosition(2300);
        SliderD.setTargetPosition(2300);
        SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderS.setPower(0.5);
        SliderD.setPower(0.5);
        while(SliderS.isBusy() && SliderD.isBusy() && !isStopRequested())
        {
            telemetry.addData("pozitia brat", SliderS.getCurrentPosition());
            telemetry.addData("pozitia brat2", SliderD.getCurrentPosition());
            telemetry.update();
        }
         */

    }
}