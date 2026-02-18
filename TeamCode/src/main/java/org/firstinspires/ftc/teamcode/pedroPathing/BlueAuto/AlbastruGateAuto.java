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

@Autonomous(name = "AlbastruGateAuto", group = "Autonomous")
@Configurable // Panels
public class AlbastruGateAuto extends OpMode {
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
    private PathChain Path1; // To intermediate position
    private PathChain Path2; // To shooting position
    private PathChain Path3; // Prepare to pick up first set of balls
    private PathChain Path4; // Pick up balls / Push gate
    private PathChain Path5; // Return to shooting position
    private PathChain Path6; // Prepare to pick up second set of balls
    private PathChain Path7; // Pick up second 3 balls
    private PathChain Path14; // Go back a bit
    private PathChain Path8; // Go to shoot
    private PathChain Path9; // Prepare to pick up third set
    private PathChain Path10; // Pick up third set
    private PathChain Path11; // Go to shoot
    private PathChain Path12; // Park

    private final double pickupWaitTime = 0.5; // seconds
    private final double gateWaitTime = 2.3; // seconds (Requested: 2 sec when pushing gate)
    private final double maxShootingTime = 1.5; // seconds (Requested: 1.5 sec max)

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        limeLight = new LimeLight(true, true);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        shooter = new Shooter(telemetryCustom, mixer, limeLight, false);
        shooter.Initialize(hardwareMap);
        shooter.ForceUpdateShooterF();

        turret = new TurretPositionControl(limeLight, shooter);
        turret.Initialize(hardwareMap, true); // Reset encoder at start

        follower = Constants.createFollower(hardwareMap);
        // Starting pose from AlbastruScurt9BallShortPark
        follower.setStartingPose(new Pose(33.476, 134.481, Math.toRadians(90)));

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths() {
        // Path 1: From start to end of AS12Gate Path 1
        Path1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(33.476, 134.481),
                        new Pose(57.369, 124.144),
                        new Pose(59.080, 112.235)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        // prepare to pick up balls
        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(59.080, 112.235),

                        new Pose(54.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // pick up balls
        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(54.000, 85.000),

                        new Pose(18.5, 84.979)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // open gate
        Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(16.7, 84.979),
                        new Pose(30.992, 73.479),
                        new Pose(18.3, 72.214)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        // go to shoot
        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(18.3, 72.214),

                        new Pose(54.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        // prepare to pick up balls
        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(54.000, 85.000),

                        new Pose(53.225, 59.615)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // pick up balls
        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(53.225, 59.615),

                        new Pose(16.7, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // go back a bit
        Path14 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(16.7, 59.000),

                        new Pose(25.000, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // got to shoot
        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(25.000, 59.000),

                        new Pose(55.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // prepare to pick up balls
        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(55.000, 85.000),

                        new Pose(49.765, 35.610)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // pick up balls
        Path10 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(49.765, 35.610),

                        new Pose(16.7, 35.471)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // go to shoot
        Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(16.6, 35.471),

                        new Pose(55.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        // park
        Path12 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(55.000, 85.000),

                        new Pose(30.000, 60.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
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
        follower.update();
        shooter.Run();
        limeLight.Run();
        turret.Run();
        // mixer.Run() removed (managed by Shooter)

        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Turret Angle", turret.getCurrentAngle());
        panelsTelemetry.debug("Shooter RPM", shooter.GetVelocityCurrent());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Move to shooting position
                mixer.Run();
                follower.followPath(Path1, true);
                follower.setMaxPower(0.85);
                setPathState(1);
                break;

            case 1:
                // Reading tags while moving (Done automatically in limeLight.Run())
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Arrived at shooting position
                mixer.Run();
                if (!follower.isBusy()) {
                    // Turret calibration sequence (Requested)
                    turret.setTargetAngle(0); // Go to middle pos
                    setPathState(20); // Calibration state
                }
                break;

            case 20:
                // Wait for turret to reach middle and calibrate - 1 sec stabilization requested
                turret.setTrackingTag(true);
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    setPathState(3); // Go to shooting
                }
                break;

            case 3:
                // Let wheels spin up (Matching RosuScurt9BallShortParkTest)
                if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                    shooter.StartAutoShoot();
                    setPathState(4);
                }
                break;

            case 4:
                // Transition when empty or timeout
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(Path4, true); // Pushing gate
                    setPathState(6);
                }
                break;

            case 6:
                // Wait for 2.3 seconds AFTER reaching arrival
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > gateWaitTime) {
                        follower.setMaxPower(0.75);
                        follower.followPath(Path5, true);
                        setPathState(7);
                    }
                } else {
                    pathTimer.resetTimer(); // Keep resetting until arrival
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    shooter.StartAutoShoot();
                    setPathState(8);
                }
                break;

            case 8:
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(Path6, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(Path7, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(Path14, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(Path8, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    shooter.StartAutoShoot();
                    setPathState(13);
                }
                break;

            case 13:
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(Path9, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(Path10, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(Path11, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    shooter.StartAutoShoot();
                    setPathState(17);
                }
                break;

            case 17:
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(Path12, true);
                    setPathState(18);
                }
                break;

            case 18:
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
    }
}
