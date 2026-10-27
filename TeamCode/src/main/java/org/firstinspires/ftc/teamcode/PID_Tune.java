package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PID_Tune", group = "Linear Opmode")
public class PID_Tune extends LinearOpMode {

    private GamepadEx ct1;
    private Tunare_PID tunarepid;

    private void Initialize() {
        ct1 = new GamepadEx(gamepad1);
        tunarepid = new Tunare_PID(ct1, telemetry);
        tunarepid.Initialize(hardwareMap);
    }

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        while (opModeIsActive()) {
            tunarepid.Run();
            idle();
        }
    }
}