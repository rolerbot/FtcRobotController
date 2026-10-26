package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// OpMode doar pentru shooter, controlat de gamepad1. Controalele sunt descrise in FlywheelShooter.
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "FlywheelTuning", group = "Linear Opmode")
public class FlywheelTuning extends LinearOpMode {
    private FlywheelShooter shooter;

    public void runOpMode() {
        shooter = new FlywheelShooter(new GamepadEx(gamepad1), telemetry);
        shooter.Initialize(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            shooter.Run();
            telemetry.update();
        }
        shooter.Stop();
        telemetry.update();
    }
}
