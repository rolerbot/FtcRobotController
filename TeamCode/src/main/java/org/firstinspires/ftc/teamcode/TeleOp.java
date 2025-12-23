package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv_pipelines.ExamplePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    Drivetrain drivetrain;
    Intake intake;
    Mixer mixer;
    Shooter shooter;
    OpenCvCamera camera;

    private void Initialize() {
        MapControlerButtons();
        drivetrain = new Drivetrain(ct1, ct2);
        drivetrain.Initialize(hardwareMap);
        drivetrain.schimbator = 1.4 - drivetrain.schimbator;

        intake = new Intake(ct1);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(intake);
        mixer.Initialize(hardwareMap);

        shooter = new Shooter(mixer,intake,ct1);
        shooter.Initialize(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId
                );

        camera.setPipeline(new ExamplePipeline());

        camera.openCameraDevice();
        camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }

    public void runOpMode()
    {
        Initialize();
        waitForStart();
        while (opModeIsActive())
        {
            intake.Run();
            drivetrain.Run();
            mixer.Run();
            shooter.Run();
            telemetry.update();
        }
    }
}