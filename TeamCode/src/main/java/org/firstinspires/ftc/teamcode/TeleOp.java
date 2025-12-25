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

        huskyLens = new Husky();
        huskyLens.Initialize(hardwareMap);
    }

    public void runOpMode()
    {
        Initialize();
        waitForStart();
        left = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        right = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        while (opModeIsActive())
        {
            Utils.TelemReset();
            huskyLens.Run();
            intake.Run();
            drivetrain.Run();
            if(!shooter.GetIsShooting())
                mixer.Run();
            shooter.Run();
//            Utils.Telem(telemetry, "PozLever", shooter.GetPositionLever());
//            Utils.Telem(telemetry, "ColorB", mixer.GetColorBlue());
//            Utils.Telem(telemetry, "ColorG", mixer.GetColorGreen());
//            Utils.Telem(telemetry, "ColorR", mixer.GetColorRed());
//            Utils.Telem(telemetry, "Husky ID:" , huskyLens.GetID());

            Utils.TelemUpdate(telemetry);
        }
    }
}