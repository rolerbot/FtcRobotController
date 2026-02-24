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

@Autonomous(name = "AS6NoGate", group = "Autonomous")
@Configurable // Panels
public class AS6NoGate extends OpMode {
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

    private Paths paths; // Paths defined in the Paths class

    private final double maxShootingTime = 3; // seconds
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

        mixer.Reset(); // Ensure mixer is at home and encoder is zero

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.5, 121.6, Math.toRadians(234)));

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
        public PathChain Path2;
        public PathChain Path4;
        public PathChain Path12;

        public Paths(Follower follower) {

            // position to shoot (Preload)
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(19.5, 121.6),
                            new Pose(54, 85)))
                    .setLinearHeadingInterpolation(Math.toRadians(234), Math.toRadians(180))
                    .build();

            // pick up Spike 1
            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(54.000, 85.000),
                            new Pose(21, 85)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // go to shoot Spike 1 (Direct line, No Gate)
            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(21, 85),
                            new Pose(54.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // park
            Path12 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(54.000, 85.000),
                            new Pose(30.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Shot 1: Preload Scan & Move
                follower.setMaxPower(1);
                follower.followPath(paths.Path1, true);
                limeLight.getLimelight().pipelineSwitch(0);
                turret.setTrackingTag(false);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(3);
                }
                break;

            case 3:
                turret.setTrackingTag(false);
                turret.setTargetAngle(-45);
                setPathState(4);
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.4
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 1)) {
                    limeLight.RelocalizationBlue();
                    turret.setTrackingTag(true);
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    shooter.StartAutoShoot();
                    setPathState(6);
                }
                break;

            case 6:
                // Finish Preload, Start Spike 1 Pickup
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path2, true);
                    follower.setMaxPower(0.7);
                    setPathState(100);
                }
                break;

            case 100:
                // Picking Spike 1
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path4, true);
                        setPathState(8);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 8:
                // Shot 2: Spike 1
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        shooter.StartAutoShoot();
                        setPathState(18); // Ready to Park
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 18:
                // Finish Spike 1 Shot and Park
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path12, true);
                    setPathState(19);
                }
                break;

            case 19:
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
