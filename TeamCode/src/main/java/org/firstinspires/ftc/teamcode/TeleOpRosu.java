package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RobotRosu", group = "Linear Opmode")
public class TeleOpRosu extends LinearOpMode {
    TelemetryCustom myLogger;
    Drivetrain drivetrain;
    Intake intake;
    Mixer mixer;
    Shooter shooter;
    LimeLight limelight;
    TurretPositionControl turretMechanism;
    RobotPinpoint robotPinpoint;
    private GamepadEx ct1, ct2;
    ButtonReader resetMixer;

    private void MapControlerButtons() {
        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);
    }

    private void Initialize() {
        myLogger = new TelemetryCustom(telemetry);
        MapControlerButtons();
    }

    private void InitAfter() {
        drivetrain = new Drivetrain(ct1, ct2);
        drivetrain.Initialize(hardwareMap);
        drivetrain.schimbator = 1.4 - drivetrain.schimbator;

        // Initialize RobotPinpoint (with both controllers for rumble and ct2 for reset
        // button)
        robotPinpoint = new RobotPinpoint(myLogger, ct1, ct2);
        robotPinpoint.SetInitialPosition(9, 9, 90); // Red alliance starting position
        robotPinpoint.Initialize(hardwareMap);

        intake = new Intake(myLogger, ct1);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(myLogger, intake);
        mixer.Initialize(hardwareMap);

        limelight = new LimeLight(false, false);
        limelight.Initialize(hardwareMap);

        shooter = new Shooter(myLogger, mixer, ct1, ct2, limelight);
        shooter.Initialize(hardwareMap);

        turretMechanism = new TurretPositionControl(limelight, shooter);
        turretMechanism.Initialize(hardwareMap);

        resetMixer = new ButtonReader(ct2, GamepadKeys.Button.RIGHT_STICK_BUTTON);

    }

    public void runOpMode() {

        Initialize();
        waitForStart();
        InitAfter();

        while (opModeIsActive()) {
            robotPinpoint.Run();
            limelight.Run();
            turretMechanism.Run();
            intake.Run();
            drivetrain.Run();
            resetMixer.readValue();
            if (resetMixer.wasJustPressed()) {
                mixer.RequestManualReset();
            }
            if (!shooter.GetShootingAllow())
                mixer.Run();
            shooter.Run();

            shooter.CalculateShootingVelocityTelemetry();

            telemetry.update();
        }
        telemetry.update();
        myLogger.close();
    }
}