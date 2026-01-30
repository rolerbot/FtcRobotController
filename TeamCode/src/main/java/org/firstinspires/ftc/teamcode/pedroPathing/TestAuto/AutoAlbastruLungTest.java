package org.firstinspires.ftc.teamcode.pedroPathing.TestAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@Autonomous(name = "AutoAlbastruLungTest", group = "Autonomous")
@Configurable // Panels
public class AutoAlbastruLungTest extends OpMode {
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
    private PathChain Path1; // Go to tag
    private PathChain Path2; // Return to shooting position
    private PathChain Path3; // Curve to field balls
    private PathChain Path4; // Pickup 3 field balls (straight line)
    private PathChain Path5; // Return to shoot after field balls
    private PathChain Path6; // Go toward human player
    private PathChain Path7; // Final approach to human player
    private PathChain Path8; // Return to shoot after human player
    private PathChain Path9; // Park

    private final double pickupWaitTime = 0.5; // seconds - wait time for field ball pickup
    private final double humanPlayerWaitTime = 1.0; // seconds - wait time at human player

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
        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null, true);
        shooter.Initialize(hardwareMap);

        // NOW create follower and set starting pose AFTER subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57.000, 9.000, Math.toRadians(90)));

        buildPaths(); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Mixer", "3 balls loaded");
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths() {
        // Path 1: Go to tag
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.000, 9.000),
                                new Pose(57.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        // Path 2: Return to shooting position
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.000, 35.000),
                                new Pose(56.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                .build();

        // Path 3: Curve to field balls
        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 15.000),
                                new Pose(66.582, 33.665),
                                new Pose(50.701, 36.060)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        // Path 4: Pickup 3 field balls (straight line)
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.701, 36.060),
                                new Pose(15.131, 36.191)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        // Path 5: Return to shoot after field balls
        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.131, 36.191),
                                new Pose(56.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                .build();

        // Path 6: Go toward human player
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 15.000),
                                new Pose(24.657, 12.367)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                .build();

        // Path 7: Final approach to human player
        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.657, 12.367),
                                new Pose(11.869, 12.135)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        // Path 8: Return to shoot after human player
        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.869, 12.135),
                                new Pose(56.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                .build();

        // Path 9: Park
        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 15.000),
                                new Pose(37.207, 12.809)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(90))
                .build();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.SetPowerMax();
        shooter.Run();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing

        // Update subsystem
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
                // Go to tag
                follower.followPath(Path1, true);
                mixer.Run();
                setPathState(1);
                break;

            case 1:
                // Wait to reach tag
                if (!follower.isBusy()) {
                    mixer.Run();
                    follower.setMaxPower(0.5); // Slower when going back from tag
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for return to shoot position
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8); // Reset to normal power
                    panelsTelemetry.debug("Status", "At shoot position - shooter spinning");
                    pathTimer.resetTimer();
                    mixer.Run();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f RPM", currentRPM, targetRPM));

                // When RPM is close to target OR timeout, start shooting
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    panelsTelemetry.debug("Status", "Shooting preload!");
                    shooter.StartAutoShoot(); // Start autonomous shooting
                    shooter.Run();
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                shooter.Run();
                // Wait for shooting to complete (3 preload balls)
                if (!shooter.IsShootingStateNone() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    panelsTelemetry.debug("Status", "Done shooting - going to field balls");
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                mixer.Run();
                // Wait for curve path to complete
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Starting field ball pickup");
                    follower.setMaxPower(0.25); // Slow for ball pickup
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Collecting 3 balls from field while moving
                follower.setMaxPower(0.25);
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Field balls collected - brief wait");
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                mixer.Run();
                // Brief wait to ensure balls are collected
                if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                    panelsTelemetry.debug("Status", "Returning to shoot");
                    follower.setMaxPower(0.8);
                    follower.followPath(Path5, true);
                    setPathState(8);
                }
                break;

            case 8:
                // Return to shoot position after field balls
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - shooting field balls");
                    shooter.StartAutoShoot(); // Start second shooting (3 field balls)
                    shooter.Run();
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait for second shooting to complete
                shooter.Run();
                if (!shooter.IsShootingStateNone() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    panelsTelemetry.debug("Status", "Done shooting - going to human player");
                    follower.followPath(Path6, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Going toward human player
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Near human player - final approach");
                    follower.setMaxPower(0.2); // Very slow for human player approach
                    follower.followPath(Path7, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Final approach to human player
                mixer.Run();
                follower.setMaxPower(0.2);

                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "At human player - collecting");
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                // Wait at human player for ball pickup
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > humanPlayerWaitTime) {
                    panelsTelemetry.debug("Status", "Human balls collected - returning");
                    follower.setMaxPower(0.8);
                    follower.followPath(Path8, true);
                    setPathState(13);
                }
                break;

            case 13:
                // Return to shoot position after human player
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - final shooting");
                    shooter.StartAutoShoot(); // Start third shooting (human player balls)
                    shooter.Run();
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                // Wait for third shooting to complete
                shooter.Run();
                if (!shooter.IsShootingStateNone() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    panelsTelemetry.debug("Status", "Done final shooting - parking");
                    intake.StopMotor();
                    follower.followPath(Path9, true);
                    setPathState(15);
                }
                break;

            case 15:
                // Park and finish
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "COMPLETE - 9 BALLS SCORED!");
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
    }
}