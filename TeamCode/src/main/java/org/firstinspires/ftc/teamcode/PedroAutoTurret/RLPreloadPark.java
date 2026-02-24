package org.firstinspires.ftc.teamcode.PedroAutoTurret;

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
import org.firstinspires.ftc.teamcode.TurretPositionControl;

@Autonomous(name = "RLPreloadPark", group = "Autonomous")
@Configurable
public class RLPreloadPark extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private LimeLight limeLight;
    private Shooter shooter;
    private Mixer mixer;
    private TurretPositionControl turret;

    // Paths
    private Paths paths;

    private final double maxShootingTime = 3; // seconds

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        // Initialize LimeLight for Red alliance
        limeLight = new LimeLight(false, true);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls (preload)

        shooter = new Shooter(telemetryCustom, mixer, limeLight, false);
        shooter.Initialize(hardwareMap);
        shooter.ForceUpdateShooterF();

        turret = new TurretPositionControl(limeLight, shooter);
        turret.Initialize(hardwareMap, true);
        turret.setUsePinpointFallback(false);
        turret.setTargetTicks(916);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88.000 - 9, 9, Math.toRadians(0)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
        follower.update();
        shooter.Run();
        limeLight.Run();
        turret.Run();

        autonomousPathUpdate();

        // Log values
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("Turret Angle", turret.getCurrentAngle());
        panelsTelemetry.debug("Shooter RPM", shooter.GetVelocityCurrent());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain PathToShoot;
        public PathChain PathToStart;
        public PathChain PathToPark;

        public Paths(Follower follower) {
            // Go to shooting position
            PathToShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(88.000 - 9, 9.000),
                            new Pose(99.000 - 9, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Return to starting position
            PathToStart = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(99.000 - 9, 17.000),
                            new Pose(88.000 - 9, 9.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Park
            PathToPark = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(88.000 - 9, 9.000),
                            new Pose(112.000 - 9, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Sit for 1 second reading tag with Pipeline 0
                follower.setMaxPower(0.45);
                limeLight.getLimelight().pipelineSwitch(0);

                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 1.0) {
                    if (limeLight.GetID() == 0) {
                        limeLight.SkipArtifactDetection();
                    }

                    // Switch to Red pipeline
                    limeLight.RelocalizationRed();
                    turret.setTrackingTag(true);
                    follower.setMaxPower(0.40);
                    follower.followPath(paths.PathToShoot, true);
                    setPathState(1);
                }
                break;

            case 1:
                // Move to shooting position + stabilize
                mixer.Run();
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        follower.setMaxPower(0.7);
                        shooter.StartAutoShoot();
                        setPathState(2);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 2:
                // Shoot preload
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.PathToStart, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Return to starting position
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetMotorPower(0.0);
                    setPathState(4);
                }
                break;

            case 4:
                // Wait until 25 seconds remain (5 seconds elapsed in a 30s auto)
                if (opmodeTimer.getElapsedTimeSeconds() >= 25.0) {
                    follower.followPath(paths.PathToPark, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Park
                if (!follower.isBusy()) {
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

