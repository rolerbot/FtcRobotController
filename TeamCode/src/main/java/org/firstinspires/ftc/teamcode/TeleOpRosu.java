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
    TurretMechanismTutorial turretMechanism;
    RobotPinpoint robotPinpoint;
    ButtonReader left;
    ButtonReader right;
    private GamepadEx ct1, ct2;

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

        shooter = new Shooter(myLogger, mixer, intake, ct1, ct2, limelight);
        shooter.Initialize(hardwareMap);

        turretMechanism = new TurretMechanismTutorial(limelight, shooter);
        turretMechanism.Initialize(hardwareMap);

        left = new ButtonReader(ct2, GamepadKeys.Button.DPAD_LEFT);
        right = new ButtonReader(ct2, GamepadKeys.Button.DPAD_RIGHT);

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
            if (!shooter.GetShootingAllow())
                mixer.Run();
            shooter.Run();

            // Afișează telemetria de la RobotPinpoint
            robotPinpoint.UpdateTelemetry(telemetry);

            // Afișează telemetria de la LimeLight (Pinpoint, MT1, MT2)
            limelight.UpdateTelemetry(telemetry);

            /*
             * telemetry.addLine("\n=== SHOOTER INFO ===");
             * telemetry.addData("ShoottingAllowed", !shooter.IsNotShooting());
             * 
             * telemetry.addLine("\n=== MIXER INFO ===");
             * telemetry.addData("Balls", mixer.GetArtifactCount());
             * telemetry.addData("Ball 1:", mixer.GetColorForPoz(0));
             * telemetry.addData("Ball 2:", mixer.GetColorForPoz(1));
             * telemetry.addData("Ball 3:", mixer.GetColorForPoz(2));
             * telemetry.addData("Sensor color R:", mixer.GetColorRed());
             * telemetry.addData("Sensor color G:", mixer.GetColorGreen());
             * telemetry.addData("Sensor color B:", mixer.GetColorBlue());
             */

            telemetry.update();
        }
        telemetry.update();
        myLogger.close();
    }
}