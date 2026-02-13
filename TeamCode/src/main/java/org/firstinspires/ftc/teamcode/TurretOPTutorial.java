package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp
public class TurretOPTutorial extends OpMode {
    private TurretMechanismTutorial turret;
    private LimeLight limeLight;
    private Drivetrain drivetrain;
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    double[] stepSizes = { 0.1, 0.01, 0.001, 0.0001, 0.00001 };
    int stepIndex = 2;

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        limeLight = new LimeLight(true, false);
        turret = new TurretMechanismTutorial(limeLight);
        drivetrain = new Drivetrain(gamepadEx1, gamepadEx2);
    }

    @Override
    public void start() {
        limeLight.Initialize(hardwareMap);
        drivetrain.Initialize(hardwareMap);
        // Important: Instruct the turret to reset encoder knowing it's at the RIGHT
        // barrier
        turret.Initialize(hardwareMap, true);
    }

    @Override
    public void loop() {
        limeLight.Run();
        turret.Run();
        drivetrain.Run();

        // Read updated states
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();

        // GAMEPAD 1: Standard PID Tuning & Tracking Toggle
        if (gamepadEx1.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X))
            turret.setTrackingTag(!turret.isTrackingTag());
        if (gamepadEx1.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B))
            stepIndex = (stepIndex + 1) % stepSizes.length;

        if (gamepadEx1.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT))
            turret.SetKp(turret.GetKp() - stepSizes[stepIndex]);
        if (gamepadEx1.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT))
            turret.SetKp(turret.GetKp() + stepSizes[stepIndex]);
        if (gamepadEx1.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP))
            turret.SetKd(turret.GetKd() + stepSizes[stepIndex]);
        if (gamepadEx1.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN))
            turret.SetKd(turret.GetKd() - stepSizes[stepIndex]);

        // GAMEPAD 2: Rotation PID & Heading Feedforward Tuning (kv_head)
        if (gamepadEx2.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT))
            turret.SetKpRotation(turret.GetKpRotation() - stepSizes[stepIndex]);
        if (gamepadEx2.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT))
            turret.SetKpRotation(turret.GetKpRotation() + stepSizes[stepIndex]);
        if (gamepadEx2.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP))
            turret.SetKdRotation(turret.GetKdRotation() + stepSizes[stepIndex]);
        if (gamepadEx2.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN))
            turret.SetKdRotation(turret.GetKdRotation() - stepSizes[stepIndex]);

        // kv_head (Heading Feedforward) on Bumpers - ONE BUTTON ONLY
        if (gamepadEx2.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER))
            turret.SetKvHead(turret.GetKvHead() - stepSizes[stepIndex]);
        if (gamepadEx2.wasJustPressed(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER))
            turret.SetKvHead(turret.GetKvHead() + stepSizes[stepIndex]);

        telemetry.addData("Tracking", turret.isTrackingTag() ? "ENABLED" : "DISABLED");
        telemetry.addData("Current Angle", "%.1f°", turret.getCurrentAngle());
        telemetry.addData("Current Ticks", turret.getCurrentTicks());
        telemetry.addData("Target Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("Error", "%.1f°", turret.getError());
        telemetry.addData("Motor Power", "%.2f", turret.getPower());
        telemetry.addData("Robot Turn Speed", "%.1f deg/s", limeLight.GetRobotHeadingVelocity());
        telemetry.addLine("");
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addData("Kp", turret.GetKp());
        telemetry.addData("Kd", turret.GetKd());
        telemetry.addData("Kp Rot", turret.GetKpRotation());
        telemetry.addData("Kd Rot", turret.GetKdRotation());
        telemetry.addData("KvHead", turret.GetKvHead());
        telemetry.update();
    }
}
