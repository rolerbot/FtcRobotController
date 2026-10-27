package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RobotAlbastru", group = "Linear Opmode")
public class TeleOpAlbastru extends LinearOpMode {
    TelemetryCustom myLogger;
    Drivetrain2 drivetrain;
    Intake intake;
    Mixer mixer;
    Shooter shooter;
    LimeLight limelight;
    TurretShot turretMechanism;
    RobotPinpoint robotPinpoint;
    FieldDrawing fieldDrawing;
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
        robotPinpoint = new RobotPinpoint(myLogger, ct1, ct2, true);
        robotPinpoint.SetInitialPosition(135, 9, 90); // Blue alliance starting position
        robotPinpoint.Initialize(hardwareMap);

        drivetrain = new Drivetrain2(ct1, ct2, robotPinpoint);
        drivetrain.Initialize(hardwareMap);
        drivetrain.schimbator = 1.4 - drivetrain.schimbator;

        intake = new Intake(myLogger, ct1);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(myLogger, intake);
        mixer.Initialize(hardwareMap);

        limelight = new LimeLight(true, false, ct2);
        limelight.Initialize(hardwareMap);

        shooter = new Shooter(myLogger, mixer, ct1, ct2, limelight, robotPinpoint);
        shooter.Initialize(hardwareMap);

        turretMechanism = new TurretMotionCompensatedPID(robotPinpoint, ct2); // varianta 2: tabel + tinta virtuala
        turretMechanism.Initialize(hardwareMap);
        shooter.SetTurretShot(turretMechanism);

        fieldDrawing = new FieldDrawing();
    }

    public void runOpMode() {
        Initialize();
        waitForStart();
        InitAfter();

        while (opModeIsActive()) {
            robotPinpoint.Run();
            limelight.Run();
            turretMechanism.Run();

            // Primele pe ecran: pozitia robotului si turela
            robotPinpoint.UpdateTelemetry(telemetry);
            drivetrain.UpdateTelemetry(telemetry);
            turretMechanism.UpdateTelemetry(telemetry);
            fieldDrawing.Draw(robotPinpoint, turretMechanism);

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

            telemetry.update();
        }
        telemetry.update();
        myLogger.close();
    }
}
