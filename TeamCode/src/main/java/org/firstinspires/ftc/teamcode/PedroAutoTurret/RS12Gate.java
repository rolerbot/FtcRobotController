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

@Autonomous(name = "RS12Gate", group = "Autonomous")
@Configurable // Panels
public class RS12Gate extends OpMode {
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

        // Initialize LimeLight for Red alliance (idBlue = false)
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
        // Mirrored Pose: (144-19.5, 121.6, 180-234=306) -> (124.5, 121.6, 306)
        follower.setStartingPose(new Pose(124.500, 121.600, Math.toRadians(306)));

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
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path14;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {

            // position to shoot (Mirrored X, Heading flipped)
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(124.5, 121.600),
                            new Pose(90.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(306), Math.toRadians(0))
                    .build();

            // pick up balls (Mirrored X)
            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(90.000, 85.000),
                            new Pose(123.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // open gate (Mirrored X)
            Path3 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(123.000, 85.000),
                            new Pose(114.000, 80.000),
                            new Pose(125.000, 76.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            // go to shoot (Mirrored X)
            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(125.000, 76.000),
                            new Pose(90.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            // prepare to pick up baalls (Mirrored X)
            Path5 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(90.000, 85.000),
                            new Pose(90.775, 59.615)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // pick up balls (Mirrored X)
            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(90.775, 59.615),
                            new Pose(124.000, 59.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go back a bit (Mirrored X)
            Path7 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(124.000, 59.000),
                            new Pose(119.000, 59.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // got to shoot (Mirrored X)
            Path8 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(119.000, 59.000),
                            new Pose(89.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // prepare to pick up balls (Mirrored X)
            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(89.000, 85.000),
                            new Pose(94.235, 35.610)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // pick up balls (Mirrored X)
            Path10 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(94.235, 35.610),
                            new Pose(128.000, 35.471)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // go to shoot (Mirrored X)
            Path11 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(128.000, 35.471),
                            new Pose(89.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // park (Mirrored X)
            Path12 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(91.000, 92.000),
                            new Pose(101, 80)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1);
                follower.followPath(paths.Path1, true);
                limeLight.getLimelight().pipelineSwitch(0);
                turret.setTrackingTag(false);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                    pathTimer.resetTimer();
                }
                break;

            case 2:
                if (limeLight.GetID() != 0) {
                    setPathState(3);
                } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(3);
                }
                break;

            case 3:
                turret.setTrackingTag(false);
                turret.setTargetAngle(45);
                setPathState(4);
                pathTimer.resetTimer();
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 1
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 2)) {
                    limeLight.RelocalizationBlue(); // Logic still uses RelocalizationBlue() which sets Pipeline 2
                    turret.setTrackingTag(true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path2, true);
                    follower.setMaxPower(0.7);
                    pathTimer.resetTimer();
                    setPathState(100);
                }
                break;

            case 100:
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path3, true);
                        setPathState(7);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 7:
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        shooter.StartAutoShoot();
                        setPathState(9);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 9:
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path5, true);
                    setPathState(10);
                }
                break;

            case 10:
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.Path6, true);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path7, true);
                        setPathState(12);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 12:
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    setPathState(13);
                }
                break;

            case 13:
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        shooter.StartAutoShoot();
                        setPathState(14);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 14:
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path9, true);
                    setPathState(15);
                }
                break;

            case 15:
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.Path10, true);
                    setPathState(16);
                }
                break;

            case 16:
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path11, true);
                        setPathState(17);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 17:
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        shooter.StartAutoShoot();
                        setPathState(18);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 18:
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path12, true);
                    setPathState(19);
                }
                break;

            case 19:
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
