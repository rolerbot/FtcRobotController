package org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "AlbastruLungShotHumanPark", group = "Autonomous")
@Configurable // Panels
public class AlbastruLungShotHumanPark extends OpMode {
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
    private PathChain Path1; // Tag reading
    private PathChain Path2; // Shooting position
    private PathChain Path3; // Human player ball pick up
    private PathChain Path4; // Return to shooting position
    private PathChain Path5; // Parking

    private final double pickupWaitTime = 1.0; // seconds - wait time for ball pickup

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
        follower.setStartingPose(new Pose(57, 9, Math.toRadians(90)));

        buildPaths(); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Mixer", "3 balls loaded");
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths() {
        // Tag reading
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.000, 9.000),
                                new Pose(57, 40)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        // Shooting position
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57, 40),
                                new Pose(57, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                .build();

        // Human player ball pick up
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57, 15.000),
                                new Pose(13.753, 10.948)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                .build();

        // Return to shooting position
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13, 11),
                                new Pose(58.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                .build();

        // Parking
        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.000, 15.000),
                                new Pose(37.024, 15.127)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(90))
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
                // Go to tag
                follower.followPath(Path1, true);
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
                // Wait for shooting to complete
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    panelsTelemetry.debug("Status", "Done shooting - going to human player");
                    intake.SetPowerMax(); // Start intake for human player pickup
                    follower.setMaxPower(0.4); // Slower for human player approach
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Going to human player
                follower.setMaxPower(0.4);

                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "At human player - collecting balls");
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                // Wait at human player for ball pickup
                if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                    panelsTelemetry.debug("Status", "Balls collected - returning to shoot");
                    follower.setMaxPower(0.8);
                    follower.followPath(Path4, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Return to shoot position
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot - starting auto shoot");
                    shooter.StartAutoShoot(); // Start second shooting
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                // Wait for second shooting to complete
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    panelsTelemetry.debug("Status", "Done second shooting - parking");
                    intake.SetMotorPower(0.0); // Stop intake
                    follower.followPath(Path5, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Park and finish
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "COMPLETE!");
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