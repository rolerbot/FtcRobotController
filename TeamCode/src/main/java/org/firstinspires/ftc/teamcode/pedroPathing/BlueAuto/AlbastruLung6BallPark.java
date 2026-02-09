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

@Autonomous(name = "AlbastruLung6BallPark", group = "Autonomous")
@Configurable // Panels
public class AlbastruLung6BallPark extends OpMode {
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
    private PathChain Path1; // Tag reading
    private PathChain Path2; // First shooting position
    private PathChain Path3; // Curve to prepare for ball pickup
    private PathChain Path4; // Straight line ball pickup (3 balls from field)
    private PathChain Path5; // Return to shooting position
    private PathChain Path6; // Parking

    private final double pickupWaitTime = 0.5; // seconds - reduced wait time for field ball pickup

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // CRITICAL: Initialize subsystems FIRST, before creating follower
        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        limeLight = new LimeLight(true, true);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        // Pass null for robotAllignment - no IMU conflicts!
        shooter = new Shooter(telemetryCustom, mixer, intake, limeLight, true);
        shooter.Initialize(hardwareMap);

        // NOW create follower and set starting pose AFTER subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57, 9, Math.toRadians(90)));

        buildPaths(); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Mixer", "3 balls loaded");
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths() {

        //read tag
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
                                new Pose(53, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110.5))
                .build();

        // Path 3: Curve to field balls
        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54, 15.000),
                                new Pose(55, 33.665),
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
                                new Pose(54, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110.5))
                .build();

        // Parking
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54, 15.000),
                                new Pose(37.175, 11.777)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110.5), Math.toRadians(90))
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
                // Go to tag
                follower.followPath(Path1, true);
                intake.SetMotorPower(0.8);
                setPathState(1);
                break;

            case 1:
                // Wait to reach tag
                if (!follower.isBusy()) {
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
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f RPM", currentRPM, targetRPM));

                // When RPM is close to target OR timeout, start shooting
                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 2.0) {
                    panelsTelemetry.debug("Status", "Shooting!");
                    shooter.StartAutoShoot(); // Start autonomous shooting
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for shooting to complete (3 balls)
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    panelsTelemetry.debug("Status", "Done shooting - going to field balls");
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Wait for curve path to complete
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Starting ball pickup");
                    intake.SetPowerMax(); // Start intake for field ball pickup
                    follower.setMaxPower(0.25); // Slow for ball pickup
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Collecting 3 balls from field while moving
                follower.setMaxPower(0.25);

                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Balls collected - brief wait");
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
                // Return to shoot position
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - starting auto shoot");
                    shooter.StartAutoShoot(); // Start second shooting (3 more balls)
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait for second shooting to complete
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    panelsTelemetry.debug("Status", "Done second shooting - parking");
                    intake.SetMotorPower(0.0); // Stop intake
                    follower.followPath(Path6, true);
                    setPathState(10);
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