package org.firstinspires.ftc.teamcode.PedroAutoTurret;

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
import org.firstinspires.ftc.teamcode.TurretPositionControl;

@Autonomous(name = "RL11CazBun", group = "Autonomous")
@Configurable // Panels
public class RL11CazBun extends OpMode {
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
    private TurretPositionControl turret;

    // Paths
    private Paths paths;

    private final double maxShootingTime = 3;
    private final double pickupWaitTime = 0.5; // seconds

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        // Initialize LimeLight for Red alliance (isBlue = false)
        limeLight = new LimeLight(false, true);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        shooter = new Shooter(telemetryCustom, mixer, limeLight, false);
        shooter.Initialize(hardwareMap);
        shooter.ForceUpdateShooterF();

        turret = new TurretPositionControl(limeLight, shooter);
        turret.Initialize(hardwareMap, true); // Reset encoder at start (Right barrier = 0)
        turret.setUsePinpointFallback(false); // Disable odometry fallback in Auto
        turret.setTargetTicks(916); // For Red: Initialize at 0 then move to the other extreme (Left barrier)

        follower = Constants.createFollower(hardwareMap);
        // Mirrored Pose: (144-56, 9.0, 180-180=0) -> (88, 9, 0)
        follower.setStartingPose(new Pose(88.000, 9.000, Math.toRadians(0)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.SetMotorPower(0.8);
        shooter.StartAutoBoost(); // Optimized spin-up
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.Run();
        limeLight.Run();
        turret.Run();

        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Turret Angle", turret.getCurrentAngle());
        panelsTelemetry.debug("Shooter RPM", shooter.GetVelocityCurrent());
        telemetryCustom.Log("Distanta", limeLight.GetDistanceToTarget());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path5Extra;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path7Extra;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path10Extra;
        public PathChain Path11;

        public Paths(Follower follower) {
            // Go to shooting pos (Mirrored X, Heading flipped)
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(88.000, 9.000),
                            new Pose(99.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Prepare to pick up Balls (Mirrored X)
            Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(99.000, 17.000),
                            new Pose(83.382, 35.440),
                            new Pose(97.522, 35.594)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Pick up balls (Mirrored X)
            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(97.522, 35.594),
                            new Pose(131.000, 35.869)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go to shooting pose (Mirrored X)
            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(131.000, 35.869),
                            new Pose(99.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go to human player (Mirrored X)
            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(99.000, 17.000),
                            new Pose(108.924, 12.950),
                            new Pose(134.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Path 5 Extra: slow crawl (Mirrored X)
            Path5Extra = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(134.000, 12.000),
                            new Pose(137.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go to shooting pose (Mirrored X)
            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(137.000, 12.000),
                            new Pose(99.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go to humanplayer closer (Mirrored X)
            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(99.000, 17.000),
                            new Pose(104.853, 12.484),
                            new Pose(134.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Path 7 Extra: slow crawl (Mirrored X)
            Path7Extra = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(134.000, 12.000),
                            new Pose(137.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go to shooting pose (Mirrored X)
            Path8 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(137.000, 12.000),
                            new Pose(99.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go to humanplayer even closer (Mirrored X)
            Path10 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(99.000, 17.000),
                            new Pose(102.000, 12.000),
                            new Pose(134.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Path 10 Extra: slow crawl (Mirrored X)
            Path10Extra = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(134.000, 12.000),
                            new Pose(137.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go to shooting pose (Mirrored X)
            Path11 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(137.000, 12.000),
                            new Pose(99.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Park (Mirrored X)
            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(99.000, 17.000),
                            new Pose(120.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Primary case: Sit for 1 second reading tag with Pipeline 0
                follower.setMaxPower(0.5);
                limeLight.getLimelight().pipelineSwitch(0);

                // If we see any artifact tag (21-23) OR we hit the 1s timeout
                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 1.0) {
                    // Force skip if we didn't see it (for timeout)
                    if (limeLight.GetID() == 0) {
                        limeLight.SkipArtifactDetection();
                    }

                    // Switch to Pipeline 1 (Red) and start moving
                    limeLight.RelocalizationRed();
                    turret.setTrackingTag(true);
                    //follower.setMaxPower(0.40); // SLOW ARRIVAL TO BREAK BETTER
                    follower.followPath(paths.Path1, true);
                    setPathState(1);
                }
                break;

            case 1:
                // Move to first shooting position + 0.5s stabilizer
                mixer.Run();
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.8) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        shooter.StartAutoShoot();
                        setPathState(2);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 2:
                // Shooting first setup
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path2, true);
                    follower.setMaxPower(0.7);
                    setPathState(3);
                }
                break;

            case 3:
                // Prepare to pick up balls
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.Path3, true);
                    follower.setMaxPower(0.7);
                    setPathState(4);
                }
                break;

            case 4:
                // Picking up balls
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                       // follower.setMaxPower(0.4); // SLOW ARRIVAL TO BREAK BETTER
                        follower.followPath(paths.Path4, true);
                        setPathState(5);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 5:
                // Go to shooting position after pickup + 0.5s stabilizer
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        shooter.StartAutoShoot();
                        pathTimer.resetTimer();
                        setPathState(6);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 6:
                // Shooting second setup
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path5, true);
                    pathTimer.resetTimer();
                    //follower.setMaxPower(0.6);
                    setPathState(7);
                }
                break;

            case 7:
                // Reach Human Player, then Wait
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) { // HP Wait time
                        //follower.setMaxPower(0.4); // Slow nudge forward
                        follower.followPath(paths.Path5Extra, true);
                        setPathState(8);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 8:
                // Finish nudge, then go to shooting position
                mixer.Run();
                if (!follower.isBusy()) {
                    //follower.setMaxPower(0.4); // SLOW ARRIVAL TO BREAK BETTER
                    follower.followPath(paths.Path6, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Stabilize and Shoot
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        shooter.StartAutoShoot();
                        pathTimer.resetTimer();
                        setPathState(10);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 10:
                // Finish shooting, go to second HP cycle
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path7, true);
                   // follower.setMaxPower(0.6);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                // Reach Human Player 2, then Wait
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) { // HP Wait time (1s)
                       // follower.setMaxPower(0.4); // Slow nudge forward
                        follower.followPath(paths.Path7Extra, true);
                        setPathState(12);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 12:
                // Finish nudge, then go to shooting position
                mixer.Run();
                if (!follower.isBusy()) {
                   // follower.setMaxPower(0.4); // SLOW ARRIVAL TO BREAK BETTER
                    follower.followPath(paths.Path8, true);
                    setPathState(13);
                }
                break;

            case 13:
                // Stabilize and Shoot
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        shooter.StartAutoShoot();
                        pathTimer.resetTimer();
                        setPathState(14);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 14:
                // Finish Shooting second set, go to THIRD HP cycle
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path10, true);
                  //  follower.setMaxPower(0.6);
                    pathTimer.resetTimer();
                    setPathState(16);
                }
                break;

            case 15:
                // Final Parking
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetMotorPower(0.0);
                    setPathState(-1);
                }
                break;

            case 16:
                // Reach Human Player 3, then Wait
                mixer.Run();
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.SetPowerMax();
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) { // HP Wait time (1s)
                       // follower.setMaxPower(0.4); // Slow nudge forward
                        follower.followPath(paths.Path10Extra, true);
                        setPathState(17);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 17:
                // Finish nudge, then go to shooting position 3
                mixer.Run();
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.setMaxPower(0.4); // SLOW ARRIVAL TO BREAK BETTER
                    follower.followPath(paths.Path11, true);
                    setPathState(18);
                }
                break;

            case 18:
                // Stabilize and Shoot FINAL set
                mixer.Run();
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        shooter.StartAutoShoot();
                        pathTimer.resetTimer();
                        setPathState(19);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 19:
                // Finish FINAL shooting, then Park
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path9, true);
                    follower.setMaxPower(0.8);
                    setPathState(15);
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
        if (shooter != null)
            shooter.StopShooterMotors();
    }
}
