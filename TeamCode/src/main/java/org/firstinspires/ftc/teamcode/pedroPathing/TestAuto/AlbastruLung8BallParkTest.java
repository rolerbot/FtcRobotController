package org.firstinspires.ftc.teamcode.pedroPathing.TestAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@Autonomous(name = "AlbastruLung8BallParkTest", group = "Autonomous")
@Configurable // Panels
public class AlbastruLung8BallParkTest extends OpMode {
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
    private double BASE_SHOOTER_F = 13.9;
    private double shooterF = BASE_SHOOTER_F;
    private final double shooterP = 0.15;  // Increased from 0.1 for more responsive control
    private final double shooterI = 0.0;
    private final double shooterD = 8.0;
    private ShooterVoltageHelper voltageHelper;

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

    private final double pickupWaitTime = 0; // seconds - wait time for field ball pickup
    private final double humanPlayerWaitTime = 0.2; // seconds - wait time at human player

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
        shooter.ForceUpdateShooterF();

        // NOW create follower and set starting pose AFTER subsystems
        // Blue: (57.000, 9.000, 90°)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57.000, 9.000, Math.toRadians(90)));

        buildPaths(); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Mixer", "3 balls loaded");
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths() {
        // Path 1: Go to tag
        // Blue: (57.000, 9.000, 90°) → (57.000, 35.000, 90°)
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.000, 9.000),
                                new Pose(57.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        // Path 2: Return to shooting position
        // Blue: (57.000, 35.000, 90°) → (53, 15.000, 110.5°)
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.000, 35.000),
                                new Pose(53, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110.5))
                .build();

        // Path 3: Curve to field balls
        // Blue: (53, 15.000) → (55, 33.665) → (50.701, 36.060)
        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(53, 15.000),
                                new Pose(55, 33.665),
                                new Pose(50.701, 36.060)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        // Path 4: Pickup 3 field balls (straight line)
        // Blue: (50.701, 36.060) → (15.131, 36.191)
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.701, 36.060),
                                new Pose(14, 36.191)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        // Path 5: Return to shoot after field balls
        // Blue: (15.131, 36.191, 180°) → (53, 15.000, 110.5°)
        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.131, 36.191),
                                new Pose(53, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110.5))
                .build();

        // Path 6: Go toward human player
        // Blue: (53, 15.000, 110.5°) → (24.657, 13, 189.5°)
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53, 15.000),
                                new Pose(24.657, 13)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110.5), Math.toRadians(189.5))
                .build();

        // Path 7: Final approach to human player
        // Blue: (24.657, 13, 189.5°) → (11.869, 12, 189.5°)
        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.657, 13),
                                new Pose(11.869, 12)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(189.5), Math.toRadians(189.5))
                .build();

        // Path 8: Return to shoot after human player
        // Blue: (11.869, 12, 189.5°) → (53, 15.000, 110.5°)
        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.869, 12),
                                new Pose(53, 15.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(189.5), Math.toRadians(110.5))
                .build();

        // Path 9: Park
        // Blue: (53, 15.000, 110.5°) → (37.207, 12.809, 90°)
        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53, 15.000),
                                new Pose(37.207, 12.809)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110.5), Math.toRadians(90))
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
        follower.update(); // Update Pedro Pathing

        // Update subsystems
        shooter.Run();
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
                mixer.Run();
                follower.followPath(Path1, true);
                follower.setMaxPower(0.7);
                setPathState(1);
                break;

            case 1:
                // Wait to reach tag
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7); // Slower when going back from tag
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for return to shoot position
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7); // Reset to normal power
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

                // When RPM is close to target OR timeout, start shooting
                if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                    panelsTelemetry.debug("Status", "Shooting preload!");
                    shooter.StartAutoShoot(); // Start autonomous shooting
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for shooting to complete (3 preload balls)
                if (mixer.IsEmpty()) {
                    panelsTelemetry.debug("Status", "Done shooting - going to field balls");
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Wait for curve path to complete
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Starting field ball pickup");
                    intake.SetPowerMax(); // Start intake for field ball pickup
                    follower.setMaxPower(0.45); // Slow for ball pickup
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Collecting 3 balls from field while moving
                follower.setMaxPower(0.35);
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Field balls collected - brief wait");
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                // Brief wait to ensure balls are collected
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                    panelsTelemetry.debug("Status", "Returning to shoot");
                    follower.setMaxPower(0.65);
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
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait for second shooting to complete
                if (mixer.IsEmpty()) {
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
                    intake.SetPowerMax(); // Start intake for human player
                    follower.setMaxPower(0.45); // Very slow for human player approach
                    follower.followPath(Path7, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Final approach to human player
                mixer.Run();
                follower.setMaxPower(0.25);

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
                    follower.setMaxPower(0.65); // Reset power for shooting
                    panelsTelemetry.debug("Status", "Human balls collected - returning");
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
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14:
                // Wait for third shooting to complete
                if (mixer.IsEmpty()) {
                    panelsTelemetry.debug("Status", "Done final shooting - parking");
                    intake.SetMotorPower(0.0); // Stop intake
                    follower.followPath(Path9, true);
                    setPathState(15);
                }
                break;

            case 15:
                // Park and finish
                mixer.Run();
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
