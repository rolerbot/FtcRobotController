package org.firstinspires.ftc.teamcode.pedroPathing.TestAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "RosuScurt9BallShortParkTest", group = "Autonomous")
@Configurable
public class RosuScurt9BallShortParkTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private LimeLight limeLight;
    private Shooter shooter;
    private Mixer mixer;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        limeLight = new LimeLight(true, false);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts();

        shooter = new Shooter(telemetryCustom, mixer, intake, limeLight, false);
        shooter.Initialize(hardwareMap);
        shooter.ForceUpdateShooterF();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(124.908, 120.829, Math.toRadians(36)));

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Mixer", "3 balls loaded");
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths() {
        // Path 1: First curve to intermediate position
        Path1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(124.908, 120.829),
                        new Pose(83, 130),
                        new Pose(88, 107.139)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(90))
                .build();

        // Path 2: Second line to shooting position
        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(88, 107.139),
                        new Pose(91.000, 92.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(44))
                .build();

        // Path 3: Prepare to pick up balls
        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(91.000, 92.000),
                        new Pose(96.239, 82)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))
                .build();

        // Path 4: Pick up balls
        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(96.239, 82),
                        new Pose(126.789, 82)
                )
        ).setTangentHeadingInterpolation()
                .build();

        // Path 5: Return to shooting position
        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(126.789, 83),
                        new Pose(91.000, 92.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                .build();

        // Path 6: Prepare to pick up more balls
        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(91.000, 92.000),
                        new Pose(95.359, 59)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))
                .build();

        // Path 7: Pick up more balls
        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(95.359, 59),
                        new Pose(126.311, 59)
                )
        ).setTangentHeadingInterpolation()
                .build();

        // Path 8: Go back a bit
        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(126.311, 59),
                        new Pose(114, 59)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 9: Return to shooting position
        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(114, 59),
                        new Pose(91.000, 92.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                .build();

        // Path 10: Park
        Path10 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(91.000, 92.000),
                        new Pose(101, 80)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(90))
                .build();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.SetMotorPower(0.8);
        shooter.StartAutoBoost();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        shooter.Run();
        limeLight.Run();

        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooter RPM", shooter.GetVelocityCurrent());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Path 1: First curve to intermediate
                mixer.Run();
                follower.followPath(Path1, true);
                follower.setMaxPower(0.7);
                setPathState(1);
                break;

            case 1:
                // Wait to reach intermediate position
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for return to shoot position
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    panelsTelemetry.debug("Status", "At shoot position - shooter spinning");
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();
                mixer.Run();
                panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f RPM", currentRPM, targetRPM));

                if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                    panelsTelemetry.debug("Status", "Shooting preload!");
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for shooting to complete (3 preload balls)
                if (mixer.IsEmpty()) {
                    panelsTelemetry.debug("Status", "Done shooting - going to pick up balls");
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Going to prepare for ball pickup
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Starting first ball pickup");
                    intake.SetPowerMax();
                    follower.setMaxPower(0.45);
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Collecting first set of balls
                follower.setMaxPower(0.35);
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "First balls collected - brief wait");
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                // Brief wait to ensure balls are collected
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > 0.0) {
                    panelsTelemetry.debug("Status", "Returning to shoot");
                    follower.setMaxPower(0.65);
                    follower.followPath(Path5, true);
                    setPathState(8);
                }
                break;

            case 8:
                // Return to shoot position
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - shooting first batch");
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait for second shooting to complete
                if (mixer.IsEmpty()) {
                    panelsTelemetry.debug("Status", "Done shooting - going to second batch");
                    follower.followPath(Path6, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Going to prepare for second batch
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Starting second ball pickup");
                    intake.SetPowerMax();
                    follower.setMaxPower(0.45);
                    follower.followPath(Path7, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Collecting second set of balls
                follower.setMaxPower(0.35);
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Second balls collected");
                    follower.followPath(Path8, true);
                    setPathState(12);
                }
                break;

            case 12:
                // Go back a bit
                mixer.Run();
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                // Brief wait
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > 0.0) {
                    panelsTelemetry.debug("Status", "Returning to shoot");
                    follower.setMaxPower(0.65);
                    follower.followPath(Path9, true);
                    setPathState(14);
                }
                break;

            case 14:
                // Return to shoot position after second batch
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - final shooting");
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                // Wait for third shooting to complete
                if (mixer.IsEmpty()) {
                    panelsTelemetry.debug("Status", "Done final shooting - parking");
                    intake.SetMotorPower(0.0);
                    follower.followPath(Path10, true);
                    setPathState(16);
                }
                break;

            case 16:
                // Park and finish
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "COMPLETE - 9 BALLS SCORED!");
                    intake.SetMotorPower(0.0);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        intake.SetMotorPower(0.0);
    }
}
