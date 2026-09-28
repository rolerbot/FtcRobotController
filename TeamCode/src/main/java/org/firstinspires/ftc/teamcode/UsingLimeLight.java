package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "UsingLimeLight", group = "Linear Opmode")
public class UsingLimeLight extends LinearOpMode {
    private Limelight3A limelight;
    private LLResult result;

    private void Initialize() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void runOpMode() {
        Initialize();
        waitForStart();

        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            telemetry.addData("LL", result);
            telemetry.update();
        }
        telemetry.update();
    }
}
