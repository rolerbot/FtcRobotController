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

@Autonomous(name = "AlbastruLungShotPark", group = "Autonomous")
@Configurable // Panels
public class AlbastruLungShotPark extends OpMode {
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
    private PathChain Path1; // Go see tag
    private PathChain Path2; // Go back to shooting position and shoot
    private PathChain Path4; // Go prepare for parking
    private PathChain Path3; // Park

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
        // Go see tag
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.000, 9.000),
                                new Pose(56.191, 40.590)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        // Go back to shooting position and shoot
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.191, 40.590),
                                new Pose(53.000, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110.5))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53, 15.000),
                                new Pose(37.207, 12.809)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
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
                    panelsTelemetry.debug("Status", "Done shooting!");
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Park and finish
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "COMPLETE!");
                    intake.SetMotorPower(0.0); // Stop intake
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