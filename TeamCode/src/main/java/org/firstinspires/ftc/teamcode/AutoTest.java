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

import org.checkerframework.checker.units.qual.A;


@Config
@Autonomous(name = "Samples", group = "Autonomous")
public class AutoTest extends GlobalScope {

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
                    OutakeStanga.setPosition(PozOutakeStanga[1]);
                    OutakeDreapta.setPosition(PozOutakeDreapta[1]);
                    BazaStanga.setPosition(0.2489);
                    BazaDreapta.setPosition(0.2044);
                    SliderS.setTargetPosition(0);
                    SliderD.setTargetPosition(0);
                    SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                    ServoGhearaIntake.setPosition(0.02);
                }
                timerBrat.reset();
                timerPoz.reset();
                IntakeStanga.setPosition(PozIntakeSt[1]);
                IntakeDreapta.setPosition(PozIntakeDr[1]);
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


        public class LiftDown2 implements Action
        {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if (!initialized)
                {
                    OutakeStanga.setPosition(PozOutakeStanga[1]);
                    OutakeDreapta.setPosition(PozOutakeDreapta[1]);
                    SliderS.setTargetPosition(0);
                    SliderD.setTargetPosition(0);
                    SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                    ServoGhearaIntake.setPosition(0.02);
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

        public Action liftDown2()
        {
            return new LiftDown2();
        }
    }

    public class Cleste
    {
        public class CloseClawOutake implements Action
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

        public Action closeClawOutake()
        {
            return new CloseClawOutake();
        }

        public class OpenClawOutake implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                ServoGhearaOutake.setPosition(0.15);
                if(timerBrat.seconds() > 8)
                    return false;
                else return true;
            }
        }

        public Action openClawOutake()
        {
            return new OpenClawOutake();
        }

        public class OpenClawOutake2 implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                ServoGhearaOutake.setPosition(0.15);
                if(timerBrat.seconds() > 4.6)
                    return false;
                else return true;
            }
        }
        public Action openClawOutake2()
        {
            return new OpenClawOutake2();
        }

        public class CloseClawIntake implements Action
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

        public Action closeClawIntake()
        {
            return new CloseClawIntake();
        }

        public class OpenClawIntake implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                ServoGhearaIntake.setPosition(0.2);
                if(timerPoz.seconds() > 4)
                    return false;
                else return true;
            }
        }
        public Action openClawIntake()
        {
            return new OpenClawIntake();
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
                if(timerBrat.seconds() > 7)
                    return false;
                else return true;
            }
        }

        public Action doPoz2()
        {
            return new Poz2();
        }

        public class Poz22 implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                OutakeStanga.setPosition(PozOutakeStanga[2]);
                OutakeDreapta.setPosition(PozOutakeDreapta[2]);
                //if(timerBrat.seconds() > 8)
                //ServoGhearaOutake.setPosition(0.02);
                if(timerBrat.seconds() > 4.3)
                    return false;
                else return true;
            }
        }

        public Action doPoz22()
        {
            return new Poz22();
        }

    }

    public class IntakeBrat
    {
        public class IntakePrindere implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if(timerPoz.seconds() < 1){
                    IntakeStanga.setPosition(PozIntakeSt[0]);
                    IntakeDreapta.setPosition(PozIntakeDr[0]);
                }
                if(timerPoz.seconds() > 1.2)
                    ServoGhearaIntake.setPosition(0);
                if(timerPoz.seconds() > 1.4)
                {
                    IntakeStanga.setPosition(PozIntakeSt[3]);
                    IntakeDreapta.setPosition(PozIntakeDr[3]);
                    BazaDreapta.setPosition(0.04);
                    BazaStanga.setPosition(0.08);
                }
                if(timerPoz.seconds() < 2.8)
                    return true;
                else return false;
            }
        }
        public Action Intake()
        {
            return new IntakePrindere();
        }
    }

    public class IntakeBrat2
    {
        public class IntakePrindere2 implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                ServoGhearaIntake.setPosition(0.02);
                if(timerPoz.seconds() > 3){
                    IntakeStanga.setPosition(PozIntakeSt[2]);
                    IntakeDreapta.setPosition(PozIntakeDr[2]);
                }
                if(timerPoz.seconds() > 3.2){
                    OutakeStanga.setPosition(PozOutakeStanga[0]);
                    OutakeDreapta.setPosition(PozOutakeDreapta[0]);
                }
                if(timerPoz.seconds() < 4.2)
                    return true;
                else return false;
            }
        }
        public Action Intake2()
        {
            return new IntakePrindere2();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0)); //11.8, 61.7
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift();
        Cleste claw = new Cleste();
        BratOutake bratoutake = new BratOutake();
        IntakeBrat prindereintake = new IntakeBrat();
        IntakeBrat2 prindereintake2 = new IntakeBrat2();

        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(11.7, 55))
                .turn(Math.toRadians(-45));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(10, -75));


        TrajectoryActionBuilder tabWait = drive.actionBuilder(initialPose)
                .waitSeconds(4);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(4, 0))
                .turn(Math.toRadians(28.5));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-6, 0))
                .turn(Math.toRadians(-28.5));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8.3, 0))
                .turn(Math.toRadians(45));
        ///--------------Parcare
        ///Robot alianta nu se misca
        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, -120))
                .strafeTo(new Vector2d(-12, -120));
        ///PArcare directa, robot din aliana parcat la perete
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-10.5, -120));
        ///Parcare langa perete, robot din alianta parcat diff de perete
        TrajectoryActionBuilder tab7 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, -131))
                .strafeTo(new Vector2d(-10.5, -131));


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
        ///Doar parcare mai jos
        /*Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );*/
        ///Auto doua sample + parcare
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClawOutake(),
                        tab.build(),
                        lift.liftUp(),
                        bratoutake.doPoz2(),
                        claw.openClawOutake(),
                        tab2.build(),
                        lift.liftDown(),
                        prindereintake.Intake(),
                        prindereintake2.Intake2(),
                        claw.closeClawOutake(),
                        tab3.build(),
                        lift.liftUp(),
                        bratoutake.doPoz22(),
                        claw.openClawOutake2(),
                        tab4.build(),
                        lift.liftDown2(),
                        tab5.build()
                )
        );
    }
}

