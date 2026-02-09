package org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto;

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

@Autonomous(name = "AlbastruScurt9BallLongPark", group = "Autonomous")
@Configurable // Panels
public class AlbastruScurt9BallLongPark extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private LimeLight limeLight;
    private Shooter shooter;
    private Mixer mixer;

    // Paths
    private PathChain Path1; // ✅ First curve to intermediate position
    private PathChain Path2; // ✅ Second line to shooting position
    private PathChain Path3; // Prepare to pick up first set of balls
    private PathChain Path4; // Pick up first 3 balls
    private PathChain Path5; // Return to shooting position
    private PathChain Path6; // Prepare to pick up second set of balls
    private PathChain Path7; // Pick up second 3 balls
    private PathChain Path8; // Go back a bit
    private PathChain Path9; // Return to shooting position
    private PathChain Path10; // Park

    private final double pickupWaitTime = 0; // seconds - wait time for field ball pickup

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // CRITICAL: Initialize subsystems FIRST, before creating follower
        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        limeLight = new LimeLight(true, false);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        // Pass null for robotAllignment - no IMU conflicts!
        shooter = new Shooter(telemetryCustom, mixer, intake, limeLight, false);
        shooter.Initialize(hardwareMap);

        // NOW create follower and set starting pose AFTER subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.092, 120.829, Math.toRadians(144)));

        buildPaths(); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Mixer", "3 balls loaded");
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths() {
        // ✅ Path 1: First curve to intermediate position (from PedroAutonomous)
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(19.092, 120.829),
                                new Pose(61, 130),
                                new Pose(56, 107.139)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(90))
                .build();

        // ✅ Path 2: Second line to shooting position (from PedroAutonomous)
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56, 107.139),
                                new Pose(53.000, 92.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(136))
                .build();

        // Path 3: Prepare to pick up balls
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.000, 92.000),
                                new Pose(47.761, 82)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .build();

        // Path 4: Pick up balls
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(47.761, 82),
                                new Pose(17.211, 82)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        // Path 5: Return to shooting position
        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.211, 83),
                                new Pose(53.000, 92.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();

        // Path 6: Prepare to pick up more balls
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.000, 92.000),
                                new Pose(48.641, 59)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .build();

        // Path 7: Pick up more balls
        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.641, 59),
                                new Pose(17.689, 59)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        // Path 8: Go back a bit
        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.689, 59),
                                new Pose(30, 59)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 9: Return to shooting position
        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(30, 59),
                                new Pose(53.000, 92.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();

        // Path 9: Park
        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.000, 92.000),
                                new Pose(52.956, 36.892)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .build();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.SetMotorPower(0.8);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing

        // Update subsystems
        shooter.Run();
        if (!shooter.GetShootingAllow())
            mixer.Run();
        limeLight.Run();

        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
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
                // ✅ Start first curve to intermediate position
                follower.followPath(Path1, true);
                intake.SetMotorPower(0.8);
                follower.setMaxPower(0.75);
                setPathState(1);
                break;

            case 1:
                // ✅ Wait for first curve to complete
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Intermediate position - going to shoot");
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                // ✅ Wait to reach shooting position
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "At shoot position - shooter spinning");
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f RPM", currentRPM, targetRPM));

                // When RPM is close to target OR timeout, start shooting
                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 1) {
                    panelsTelemetry.debug("Status", "Shooting preload!");
                    shooter.StartAutoShoot(); // Start autonomous shooting
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for shooting to complete (3 preload balls)
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    panelsTelemetry.debug("Status", "Done shooting - going to first field balls");
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Wait for prepare path to complete
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Starting first ball pickup");
                    intake.SetPowerMax(); // Start intake for field ball pickup
                    follower.setMaxPower(0.25); // Slow for ball pickup
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Collecting first 3 balls from field while moving
                follower.setMaxPower(0.25);

                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "First 3 balls collected - brief wait");
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                // Brief wait to ensure balls are collected
                if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                    panelsTelemetry.debug("Status", "Returning to shoot");
                    follower.setMaxPower(0.8);
                    follower.followPath(Path5, true);
                    setPathState(8);
                }
                break;

            case 8:
                // Return to shoot position after first field balls
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - shooting first field balls");
                    shooter.StartAutoShoot(); // Start second shooting (3 field balls)
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait for second shooting to complete
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 1.7) {
                    panelsTelemetry.debug("Status", "Done shooting - going to second field balls");
                    follower.followPath(Path6, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Wait for second prepare path to complete
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Starting second ball pickup");
                    intake.SetPowerMax(); // Start intake for second field ball pickup
                    follower.setMaxPower(0.25); // Slow for ball pickup
                    follower.followPath(Path7, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Collecting second 3 balls from field while moving
                follower.setMaxPower(0.25);

                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Second 3 balls collected - brief wait");
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                // Brief wait to ensure balls are collected
                if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                    panelsTelemetry.debug("Status", "Returning to shoot");
                    follower.setMaxPower(0.8);
                    follower.followPath(Path8, true);
                    setPathState(13);
                }
                break;

            case 13:
                // go back a bit
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - final shooting");
                    pathTimer.resetTimer();
                    follower.followPath(Path9, true);
                    setPathState(14);
                }
                break;
            case 14:
                // Return to shoot position after second field balls
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - final shooting");
                    shooter.StartAutoShoot(); // Start third shooting (3 more field balls)
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                // Wait for third shooting to complete
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 1.7) {
                    panelsTelemetry.debug("Status", "Done final shooting - parking");
                    intake.SetMotorPower(0.0); // Stop intake
                    follower.followPath(Path10, true);
                    setPathState(16);
                }
                break;

            case 16:
                // Park and finish
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "COMPLETE - 9 BALLS SCORED!");
                    intake.SetMotorPower(0.0); // Ensure intake off
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
        intake.SetMotorPower(0.0); // Stop intake
    }
}