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
    TurretProfiledPIDControl turretMechanism;
    RobotPinpoint robotPinpoint;
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
        // Initialize RobotPinpoint (with both controllers for rumble and ct2 for reset
        // button)
        robotPinpoint = new RobotPinpoint(myLogger, ct1, ct2);
        robotPinpoint.SetInitialPosition(9, 9, 90); // Red alliance starting position
        robotPinpoint.Initialize(hardwareMap);

        drivetrain = new Drivetrain(ct1, ct2, robotPinpoint);
        drivetrain.Initialize(hardwareMap);
        drivetrain.schimbator = 1.4 - drivetrain.schimbator;

        intake = new Intake(myLogger, ct1);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(myLogger, intake);
        mixer.Initialize(hardwareMap);

        limelight = new LimeLight(false, false, ct2);
        limelight.Initialize(hardwareMap);

        shooter = new Shooter(myLogger, mixer, ct1, ct2, limelight);
        shooter.Initialize(hardwareMap);

        turretMechanism = new TurretProfiledPIDControl(limelight, ct2);
        turretMechanism.Initialize(hardwareMap);


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

            shooter.CalculateShootingVelocityTelemetry();

            telemetry.addLine("\n--- Arranged Shooting Debug ---");
            telemetry.addData("LimeLight ID", limelight.GetID());
            telemetry.addData("CanShootArranged", shooter.CanShootArranged());
            telemetry.addData("ArtifactOrder", limelight.artifactOrder[0] + "," +
                    limelight.artifactOrder[1] + "," + limelight.artifactOrder[2]);
            telemetry.addData("Mixer slot 0", mixer.artifacte[0].toString());
            telemetry.addData("Mixer slot 1", mixer.artifacte[1].toString());
            telemetry.addData("Mixer slot 2", mixer.artifacte[2].toString());
            telemetry.update();
        }
        telemetry.update();
        myLogger.close();
    }
}