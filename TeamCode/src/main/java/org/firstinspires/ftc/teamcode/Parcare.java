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


@Config
@Autonomous(name = "Doar_Parcare", group = "Autonomous")
public class Parcare extends GlobalScope {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0)); //11.8, 61.7
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //.waitSeconds(20)
                .strafeTo(new Vector2d(10, -75));

        waitForStart();

        if (isStopRequested()) return;

        Initialise();
        telemetry.update();
        OutakeStanga.setPosition(PozOutakeStanga[0]);
        OutakeDreapta.setPosition(PozOutakeDreapta[0]);
        BazaDreapta.setPosition(0.04);
        BazaStanga.setPosition(0.08);
        IntakeStanga.setPosition(0.649);
        IntakeDreapta.setPosition(0.6505);
        ServoRotire.setPosition(0.5);
        Parcare.setPosition(0.515);
        ///Doar parcare mai jos
        Actions.runBlocking(
                new SequentialAction(
                        tab1.build()
                )
        );
    }
}