package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter_Pretune", group = "Linear Opmode")
public class Shooter_Pretune extends LinearOpMode {

    private GamepadEx ct1;
    private FunctiePreTunare preTunare;

    private void Initialize() {
        ct1 = new GamepadEx(gamepad1);
        preTunare = new FunctiePreTunare(ct1, telemetry);
        preTunare.Initialize(hardwareMap);
    }

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        while (opModeIsActive()) {
            preTunare.Run();
        }
    }
}