package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Servo;
import org.openftc.easyopencv.OpenCvCamera;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotFTC", group="Linear Opmode")
public class TeleOp extends GlobalScope
{
    Drivetrain drivetrain;
    Intake intake;
    Mixer mixer;
    Shooter shooter;
    OpenCvCamera camera;
    Husky huskyLens;
    ButtonReader left;
    ButtonReader right;
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
        huskyLens.Run();
        waitForStart();
        left = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        right = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        while (opModeIsActive())
        {
            intake.Run();
            drivetrain.Run();
            mixer.Run();
            shooter.Run();
            telemetry.addData("PozLever", shooter.GetPositionLever());
            telemetry.addData("ColorB", mixer.GetColorBlue());
            telemetry.addData("ColorG", mixer.GetColorGreen());
            telemetry.addData("ColorR", mixer.GetColorRed());
            telemetry.addData("Husky ID:" , huskyLens.GetID());
            telemetry.update();
        }
    }
}