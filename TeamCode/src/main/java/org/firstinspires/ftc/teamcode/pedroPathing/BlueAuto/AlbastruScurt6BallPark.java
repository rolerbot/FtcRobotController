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

@Autonomous(name = "AlbastruScurt6BallPark", group = "Autonomous")
@Configurable // Panels
public class AlbastruScurt6BallPark extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private Mixer mixer;

    // Paths
    private PathChain Path1; // Move to shooting position
    private PathChain Path2; // Prepare to pick up balls
    private PathChain Path3; // Pick up balls (3 field balls)
    private PathChain Path4; // Return to shooting position
    private PathChain Path5; // Park
    private PathChain Path6;

    private final double pickupWaitTime = 0.2; // seconds - wait time for field ball pickup

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // CRITICAL: Initialize subsystems FIRST, before creating follower
        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        husky = new Husky(telemetryCustom);
        husky.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        // Pass null for robotAllignment - no IMU conflicts!
        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null, false);
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

        // Path 5: Move to parking position
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.000, 92.000),
                                new Pose(47.876, 69.709)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(90))
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
        husky.Run();

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
                // Go to shooting position
                follower.followPath(Path1, true);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy())
                {
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait to reach shooting position
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "At shoot position - shooter spinning");
                    setPathState(2);
                }
                break;

            case 3:
                // Wait for shooter to spin up
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f RPM", currentRPM, targetRPM));

                // When RPM is close to target OR timeout, start shooting
                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 2.0) {
                    panelsTelemetry.debug("Status", "Shooting preload!");
                    shooter.StartAutoShoot(); // Start autonomous shooting
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 4:
                // Wait for shooting to complete (3 preload balls)
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    panelsTelemetry.debug("Status", "Done shooting - going to field balls");
                    follower.followPath(Path2, true);
                    setPathState(4);
                }
                break;

            case 5:
                // Wait for prepare path to complete
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Starting field ball pickup");
                    intake.SetPowerMax(); // Start intake for field ball pickup
                    follower.setMaxPower(0.25); // Slow for ball pickup
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 6:
                // Collecting 3 balls from field while moving
                follower.setMaxPower(0.25);

                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Field balls collected - brief wait");
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 7:
                // Brief wait to ensure balls are collected
                if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                    panelsTelemetry.debug("Status", "Returning to shoot");
                    follower.setMaxPower(0.8);
                    follower.followPath(Path4, true);
                    setPathState(7);
                }
                break;

            case 8:
                // Return to shoot position after field balls
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - shooting field balls");
                    shooter.StartAutoShoot(); // Start second shooting (3 field balls)
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 9:
                // Wait for second shooting to complete
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    panelsTelemetry.debug("Status", "Done shooting - parking");
                    intake.SetMotorPower(0.0); // Stop intake
                    follower.followPath(Path5, true);
                    setPathState(9);
                }
                break;

            case 10:
                // Park and finish
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "COMPLETE - 6 BALLS SCORED!");
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