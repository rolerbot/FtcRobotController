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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@Autonomous(name = "BlueTest", group = "Autonomous")
public class AutonomTest extends GlobalScope {

    private ElapsedTime timerPoz = new ElapsedTime();
    private ElapsedTime timerBrat = new ElapsedTime();
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
                    SliderS.setTargetPosition(2400);
                    SliderD.setTargetPosition(2400);
                    SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setPower(1);
                    SliderS.setPower(1);
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
                    OutakeStanga.setPosition(PozOutakeStanga[0]);
                    OutakeDreapta.setPosition(PozOutakeDreapta[0]);
                    SliderS.setTargetPosition(0);
                    SliderD.setTargetPosition(0);
                    SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                timerBrat.reset();
                timerPoz.reset();
                double pos = SliderS.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 30)
                    return true;
                else
                {
                    //SliderS.setPower(0);
                    //SliderD.setPower(0);
                    timerPoz.startTime();
                    timerBrat.startTime();
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
                timerBrat.reset();
                timerPoz.reset();
                timerPoz.startTime();
                timerBrat.startTime();
                ServoGhearaOutake.setPosition(0.0056);
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
                ServoGhearaOutake.setPosition(0.2);
                if(timerBrat.seconds() > 9.2)
                    return false;
                else return true;
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
                //if(timerBrat.seconds() > 8)
                //ServoGhearaOutake.setPosition(0.02);
                if (timerBrat.seconds() > 7.7)
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
                if(timerPoz.seconds() > 9.5)
                    return false;
                else return true;
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
                .strafeTo(new Vector2d(13.5, 55))
                .turn(Math.toRadians(-45));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(10, -75));


        TrajectoryActionBuilder tabWait = drive.actionBuilder(initialPose)
                .waitSeconds(4);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(2, 0))
                .turn(Math.toRadians(28));



        waitForStart();

        if (isStopRequested()) return;

        Initialise();
        telemetry.addData("pozitia brat", SliderS.getCurrentPosition());
        telemetry.addData("pozitia brat2", SliderD.getCurrentPosition());
        telemetry.addData("timpBratOutake", timerBrat);
        telemetry.addData("timpBratOutake0", timerPoz);
        telemetry.update();
        OutakeStanga.setPosition(PozOutakeStanga[0]);
        OutakeDreapta.setPosition(PozOutakeDreapta[0]);
        BazaDreapta.setPosition(0.04);
        BazaStanga.setPosition(0.08);
        IntakeStanga.setPosition(0.649);
        IntakeDreapta.setPosition(0.6505);
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        tab.build(),
                        lift.liftUp(),
                        bratoutake.doPoz2(),
                        claw.openClaw(),
                        tab2.build(),
                        //bratoutake.doPoz0(),
                        lift.liftDown()
                )
        );
    }
}