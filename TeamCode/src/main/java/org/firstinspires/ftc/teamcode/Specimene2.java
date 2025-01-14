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
public class Specimene2 extends GlobalScope {

    private ElapsedTime timer = new ElapsedTime();
    int SLiderUp1 = 530, SliderUp2 = 1155, cnt;
    private double ArrayForSeconds[] = {2.2, 7};
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
                if(timer.seconds() > ArrayForSeconds[cnt])
                    ServoGhearaOutake.setPosition(0.24);
                if(timer.seconds() < ArrayForSeconds[cnt] + 0.2)
                    return true;
                else{
                    cnt++;
                    return false;
                }
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
                    OutakeDreapta.setPosition(PozOutakeDreapta[1]);
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
                OutakeDreapta.setPosition(PozOutakeDreapta[4]);
                OutakeStanga.setPosition(PozOutakeStanga[4]);
                if(timer.seconds() < 4)
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
                if(timer.seconds() < 3.5)
                    return true;
                else return false;

            }
        }

        public Action openClawOutake()
        {
            return new Cleste.OpenClawOutake();
        }
    }

    public class BratOutake
    {
        public class Brat implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                OutakeDreapta.setPosition(PozOutakeDreapta[4]);
                OutakeStanga.setPosition(PozOutakeStanga[4]);
                if(timer.seconds() < 7)
                    return true;
                else return false;

            }
        }

        public Action brat()
        {
            return new BratOutake.Brat();
        }

    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0)); //11.8, 61.7
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift();
        Cleste cleste = new Cleste();
        BratOutake bratoutake = new BratOutake();

        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-33, -33));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(5.5, 0));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8, 60))
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(23.7, 60));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(25, 65));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(25, 40))
                .turn(Math.toRadians(175))
                .strafeTo(new Vector2d(28.3, 40));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 56))//93
                .turn(Math.toRadians(-10))
                .strafeTo(new Vector2d(-25, 80))//135 56.36
                .strafeTo(new Vector2d(17, 90)) //-54.89
                .strafeTo(new Vector2d(-25, 90))//
                .strafeTo(new Vector2d(-25, 105))
                .strafeTo(new Vector2d(17, 105))
                .turn(Math.toRadians(-10))
                .strafeTo(new Vector2d(7, 56));

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
                                tab3.build()
                        ),
                        cleste.closeClawOutake(),
                        lift.liftUp1(),
                        tab4.build(),
                        lift.liftUp2(),
                        new ParallelAction(
                                lift.liftDown(),
                                tab1.build()
                        ),
                        tab5.build()
                )
        );
    }
}