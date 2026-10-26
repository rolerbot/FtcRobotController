package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RobotAlbastru", group = "Linear Opmode")
public class TeleOpAlbastru extends LinearOpMode {
    Drivetrain drivetrain;
    Intake intake;
    FlywheelShooter shooter;
    Turret turret;
    private GamepadEx ct1, ct2;

    private void MapControlerButtons() {
        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);
    }

    private void Initialize() {
        MapControlerButtons();
    }

    private void InitAfter() {

        drivetrain = new Drivetrain(ct1);
        drivetrain.Initialize(hardwareMap);
        drivetrain.schimbator = 1.4 - drivetrain.schimbator;

        intake = new Intake(ct1);
        intake.Initialize(hardwareMap);

        shooter = new FlywheelShooter(ct2, telemetry);
        shooter.Initialize(hardwareMap);

        turret = new Turret(ct2, telemetry);
        turret.Initialize(hardwareMap);
    }

    public void runOpMode() {
        Initialize();
        waitForStart();
        InitAfter();

        while (opModeIsActive()) {
            drivetrain.Run();
            intake.Run();
            shooter.Run();
            turret.Run();
            telemetry.update();
        }
        shooter.Stop();
        turret.Stop();
        telemetry.update();
    }
}
