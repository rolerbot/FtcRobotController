package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="RobotAlbastru", group="Linear Opmode")
public class TeleOpAlabstru extends LinearOpMode
{
    TelemetryCustom myLogger;
    Drivetrain drivetrain;
    Intake intake;
    Mixer mixer;
    Shooter shooter;
    RobotAlignment shooterAiming;
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

        shooterAiming = new RobotAlignment(myLogger, ct1, ct2, drivetrain,false, true, huskyLens);
        shooterAiming.Initialize(hardwareMap);

        shooter = new Shooter(myLogger, mixer, intake, huskyLens, shooterAiming ,ct1, ct2);
        shooter.Initialize(hardwareMap);

        left = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        right = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);

    }

    public void runOpMode()
    {
        Initialize();
        waitForStart();
        InitAfter();

        while (opModeIsActive())
        {
            Utils.GasirePozitii(left, right, shooter.ServoHood);
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

            telemetry.addData("ShoottingAllowed", shooter.isShooting);
            telemetry.addData("Pos X:", shooterAiming.GetRobotX());
            telemetry.addData("Pos Y:", shooterAiming.GetRobotY());
            telemetry.addData("Heading", shooterAiming.GetCurrentHeading());
            telemetry.addData("CurrntVel", shooter.GetVelocityCurrent());
            telemetry.addData("TargetVel", shooter.GetVelocityTarget());
            telemetry.addData("Balls", mixer.GetArtifactCount());
            telemetry.addData("Ball 1:", mixer.GetColorForPoz(0));
            telemetry.addData("Ball 2:", mixer.GetColorForPoz(1));
            telemetry.addData("Ball 3:", mixer.GetColorForPoz(2));
            telemetry.addData("Sensor color R:", mixer.GetColorRed());
            telemetry.addData("Sensor color G:", mixer.GetColorGreen());
            telemetry.addData("Sensor color B:", mixer.GetColorBlue());
            telemetry.update();
        }
        telemetry.update();
        myLogger.close();
    }
}