package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// OpMode doar pentru shooter, controlat de gamepad1. Controalele sunt descrise in Shooter.
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ShooterTuning", group = "Linear Opmode")
public class ShooterTuning extends LinearOpMode {
    private Shooter shooter;

    public void runOpMode() {
        shooter = new Shooter(new GamepadEx(gamepad1), telemetry);
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
