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
    RobotAllignment shooterAiming;
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

    }

    private void InitAfter()
    {
        drivetrain = new Drivetrain(ct1, ct2);
        drivetrain.Initialize(hardwareMap);
        drivetrain.schimbator = 1.4 - drivetrain.schimbator;

        huskyLens = new Husky(myLogger,ct1);
        huskyLens.Initialize(hardwareMap);

        intake = new Intake(myLogger, ct1);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(myLogger, intake);
        mixer.Initialize(hardwareMap);

        shooterAiming = new RobotAllignment(myLogger, ct1, ct2, drivetrain, false);
        shooterAiming.Initialize(hardwareMap);

        shooter = new Shooter(myLogger, mixer, intake, huskyLens, shooterAiming ,ct1, ct2);
        shooter.Initialize(hardwareMap);

    }

    public void runOpMode()
    {
        Initialize();
        waitForStart();
        InitAfter();
        left = new ButtonReader(ct1, GamepadKeys.Button.DPAD_LEFT);
        right = new ButtonReader(ct1, GamepadKeys.Button.DPAD_RIGHT);
        while (opModeIsActive())
        {
            // Utils.GasirePozitii(left, right, testServo);
            huskyLens.Run();
            intake.Run();
            drivetrain.Run();
            if(!shooter.GetShootingAllow())
                mixer.Run();
            shooter.Run();
            shooterAiming.Run();

            // Display distance to target on telemetry
            myLogger.Log("Distance to Target", String.format("%.2f m", shooterAiming.GetDistanceToTarget()));
            myLogger.Log("Robot Position", String.format("X:%.1f Y:%.1f cm", shooterAiming.GetRobotX(), shooterAiming.GetRobotY()));
            myLogger.Update();

            telemetry.addData("CurrntVel", shooter.GetVelocityCurrent());
            telemetry.addData("TargetVel", shooter.GetVelocityTarget());
            telemetry.update();
        }
        telemetry.update();
        myLogger.close();
    }
}