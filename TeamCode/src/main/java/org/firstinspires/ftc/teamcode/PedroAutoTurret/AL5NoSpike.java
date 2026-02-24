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

@Autonomous(name = "AL5NoSpike", group = "Autonomous")
@Configurable // Panels
public class AL5NoSpike extends OpMode {
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

        // Initialize LimeLight for Blue alliance
        limeLight = new LimeLight(true, true);
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
        turret.setTargetTicks(0); // Set turret to encoder position 0 (physical start) in init


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 9, Math.toRadians(180)));

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
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path5;
        public PathChain Path5Extra;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path7Extra;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            // Path 1: Start to Shoot (Preload)
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.000, 9.000),
                            new Pose(45.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 5: Shoot to HP 1
            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(45.000, 17.000),
                            new Pose(35.076, 12.950),
                            new Pose(10.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 5 Extra: slow crawl at HP
            Path5Extra = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(10.000, 12.000),
                            new Pose(7.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 6: HP 1 to Shoot
            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(7.000, 12.000),
                            new Pose(45.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 7: Shoot to HP 2
            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(45.000, 17.000),
                            new Pose(39.147, 12.484),
                            new Pose(10.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 7 Extra: slow crawl at HP
            Path7Extra = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(10.000, 12.000),
                            new Pose(7.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 8: HP 2 to Shoot
            Path8 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(7.000, 12.000),
                            new Pose(45.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 9: Final Shoot to Park
            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(45.000, 17.000),
                            new Pose(32.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Scan for 1 second
                follower.setMaxPower(0.6);
                limeLight.getLimelight().pipelineSwitch(0);
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    limeLight.RelocalizationBlue();
                    turret.setTrackingTag(true);
                    follower.followPath(paths.Path1, true);
                    setPathState(1);
                }
                break;

            case 1:
                // Move to first shoot
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        shooter.StartAutoShoot();
                        setPathState(2);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 2:
                // Shot 1 (Preload). Skip Spike (Case 3/4) and go directly to HP Cycle 1.
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path5, true);
                    follower.setMaxPower(0.6);
                    setPathState(7); // Jump to HP 1 Pickup
                }
                break;

            case 7:
                // HP 1 Pickup Sequence
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                        follower.setMaxPower(0.4);
                        follower.followPath(paths.Path5Extra, true);
                        setPathState(108); // Unique intermediate state
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 108:
                // Back to Shoot from HP 1
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.Path6, true);
                    follower.setMaxPower(0.7);
                    setPathState(9); // Move to Shot HP 1
                }
                break;

            case 9:
                // Shot 2 (HP 1)
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        shooter.StartAutoShoot();
                        pathTimer.resetTimer();
                        setPathState(10); // Ready for next cycle
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 10:
                // After Shot 2, go to HP 2
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path7, true);
                    follower.setMaxPower(0.6);
                    setPathState(11); // Start HP 2 Pickup
                }
                break;

            case 11:
                // HP 2 Pickup Sequence
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                        follower.setMaxPower(0.4);
                        follower.followPath(paths.Path7Extra, true);
                        setPathState(12);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 12:
                // Back to Shoot from HP 2
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(paths.Path8, true);
                    setPathState(13);
                }
                break;

            case 13:
                // Shot 3 (HP 2)
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        shooter.StartAutoShoot();
                        pathTimer.resetTimer();
                        setPathState(14); // Ready to park
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 14:
                // Final Park
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path9, true);
                    follower.setMaxPower(0.8);
                    setPathState(15);
                }
                break;

            case 15:
                // End
                mixer.Run();
                if (!follower.isBusy()) {
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
        if (shooter != null)
            shooter.StopShooterMotors();
    }
}
