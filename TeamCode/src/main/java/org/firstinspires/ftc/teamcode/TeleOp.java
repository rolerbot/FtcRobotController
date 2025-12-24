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
    private Servo ServoPoz1 = null;
    private Servo ServoPoz2 = null;
    ButtonReader left;
    ButtonReader right;
    private void Initialize()
    {
        MapControlerButtons();
        ServoPoz1 = hardwareMap.get(Servo.class, "ServoPoz1");
        ServoPoz1.setDirection(Servo.Direction.FORWARD);
        ServoPoz2 = hardwareMap.get(Servo.class, "ServoPoz2");
        ServoPoz2.setDirection(Servo.Direction.FORWARD);
        drivetrain = new Drivetrain(ct1, ct2);
        drivetrain.Initialize(hardwareMap);
        drivetrain.schimbator = 1.4 - drivetrain.schimbator;

        intake = new Intake(ct1);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(intake);
        mixer.Initialize(hardwareMap);

        shooter = new Shooter(mixer,intake,ct1);
        shooter.Initialize(hardwareMap);
    }

    public void runOpMode()
    {
        Initialize();
        waitForStart();
        ServoPoz1.setPosition(0.0206);
        ServoPoz2.setPosition(0.0206);
        left = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        right = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        while (opModeIsActive())
        {
            intake.Run();
            GasirePozitii1(left, right, ServoPoz1, ServoPoz2);
            drivetrain.Run();
            mixer.Run();
            shooter.Run();
            telemetry.addData("PozLever", shooter.GetPositionLever());
            telemetry.addData("ServoPoz2", ServoPoz2.getPosition());
            telemetry.addData("ServoPoz1", ServoPoz1.getPosition());
            telemetry.addData("ColorB", mixer.GetColorBlue());
            telemetry.addData("ColorG", mixer.GetColorGreen());
            telemetry.addData("ColorR", mixer.GetColorRed());
            telemetry.update();
        }
    }
}