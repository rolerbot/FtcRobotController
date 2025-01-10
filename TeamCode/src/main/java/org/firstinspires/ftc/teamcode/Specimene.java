package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name = "Specimene", group = "Autonomous")
public class Specimene extends GlobalScope {

    private ElapsedTime timer = new ElapsedTime();
    int SLiderUp1 = 610, SliderUp2 = 1200;
    public class Lift
    {
        public class LiftUp1 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if (!initialized) {
                    SliderS.setTargetPosition(SLiderUp1);
                    SliderD.setTargetPosition(SLiderUp1);
                    SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setPower(1);
                    SliderS.setPower(1);
                    initialized = true;
                    OutakeStanga.setPosition(PozOutakeStanga[2]);
                    OutakeDreapta.setPosition(PozOutakeDreapta[2]);
                }
                double pos = SliderS.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < SLiderUp1)
                    return true;
                else {
                    timer.reset();
                    timer.startTime();
                    return false;
                }
            }
        }

        public Action liftUp1()
        {
            return new Lift.LiftUp1();
        }

        public class LiftUp2 implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(timer.seconds() > 1){
                    SliderS.setTargetPosition(SliderUp2);
                    SliderD.setTargetPosition(SliderUp2);
                    SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                telemetry.addData("Brat", SliderS.getCurrentPosition());
                telemetry.addData("timp", timer.seconds());
                if(timer.seconds() > 2)
                    ServoGhearaOutake.setPosition(0.24);
                if(timer.seconds() < 2.2)
                    return true;
                else return false;
            }
        }

        public Action liftUp2()
        {
            return new Lift.LiftUp2();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    OutakeStanga.setPosition(PozOutakeStanga[1]);
                    OutakeDreapta.setPosition(PozOutakeDreapta[1]);
                    SliderS.setTargetPosition(0);
                    SliderD.setTargetPosition(0);
                    SliderS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                    OutakeStanga.setPosition(PozOutakeStanga[1]);
                    OutakeDreapta.setPosition(0.3361);
                }
                timer.reset();
                double pos = SliderS.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 30)
                    return true;
                else {
                    timer.startTime();
                    return false;
                }
            }
        }

        public Action liftDown()
        {
            return new Lift.LiftDown();
        }
    }

    public class Cleste
    {
        public class CloseClawOutake implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                ServoGhearaOutake.setPosition(0.0056);
                if(timer.seconds() < 4.3)
                    return true;
                else return false;

            }
        }

        public Action closeClawOutake()
        {
            return new Cleste.CloseClawOutake();
        }

        public class OpenClawOutake implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                ServoGhearaOutake.setPosition(0.15);
                if(timer.seconds() < 2)
                    return true;
                else return false;

            }
        }

        public Action openClawOutake()
        {
            return new Cleste.OpenClawOutake();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0)); //11.8, 61.7
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift();
        Cleste cleste = new Cleste();

        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-32.6, -33));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(5, 0));
                //.splineToLinearHeading(new Pose2d(30, 30, Math.toRadians(180)), Math.toRadians(0));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 60))
                .strafeTo(new Vector2d(-30, 60))
                .lineToXLinearHeading(80, Math.toRadians(-90))
                .strafeTo(new Vector2d(-30, 120));

        waitForStart();

        if (isStopRequested()) return;

        Initialise();

        telemetry.addData("pozitia brat", SliderS.getCurrentPosition());
        telemetry.addData("pozitia brat2", SliderD.getCurrentPosition());
        telemetry.addData("timpBratOutake0", timer);
        telemetry.update();

        OutakeStanga.setPosition(0.3725);
        OutakeDreapta.setPosition(0.4775);
        BazaDreapta.setPosition(0.04);
        BazaStanga.setPosition(0.08);
        IntakeStanga.setPosition(0.649);
        IntakeDreapta.setPosition(0.6505);
        ServoRotire.setPosition(0.5);
        Parcare.setPosition(0.515);
        ServoGhearaOutake.setPosition(0.0056);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                tab.build(),
                                lift.liftUp1()
                        ),
                        lift.liftUp2(),
                        new ParallelAction(
                                lift.liftDown(),
                                tab1.build()
                        ),
                        tab2.build()
                )
        );
    }
}