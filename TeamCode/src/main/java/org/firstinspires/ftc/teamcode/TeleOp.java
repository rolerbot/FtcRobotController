package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.openftc.easyopencv.OpenCvCamera;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends LinearOpMode
{
    TelemetryCustom myLogger;
    Drivetrain drivetrain;
    Intake intake;
    Mixer mixer;
    Shooter shooter;
    OpenCvCamera camera;
    Husky huskyLens;
    ButtonReader left;
    ButtonReader right;
    private GamepadEx ct1, ct2;
    private void MapControlerButtons()
    {
        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);
    }
    private void Initialize()
    {
        myLogger = new TelemetryCustom(telemetry);
        MapControlerButtons();

        drivetrain = new Drivetrain(ct1, ct2);
        drivetrain.Initialize(hardwareMap);
        drivetrain.schimbator = 1.4 - drivetrain.schimbator;

        huskyLens = new Husky(myLogger,ct1, drivetrain);
        huskyLens.Initialize(hardwareMap);

        intake = new Intake(myLogger, ct1);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(myLogger, intake);
        mixer.Initialize(hardwareMap);

        shooter = new Shooter(myLogger, mixer, intake, huskyLens,ct1);
        shooter.Initialize(hardwareMap);
    }

    public void runOpMode()
    {
        Initialize();
        myLogger.Log("Status", "Initialized and Ready");
        waitForStart();
        left = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        right = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        while (opModeIsActive())
        {
            huskyLens.Run();
            intake.Run();
            drivetrain.Run();
            if(shooter.IsNotShooting())
                mixer.Run();
            shooter.Run();
            myLogger.Update();
        }
        myLogger.close();
    }
}