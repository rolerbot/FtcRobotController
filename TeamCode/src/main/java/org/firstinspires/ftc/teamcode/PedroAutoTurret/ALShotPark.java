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

@Autonomous(name = "ALShotPark", group = "Autonomous")
@Configurable // Panels
public class ALShotPark extends OpMode {
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

        mixer.Reset(); // Ensure mixer is at home and encoder is zero

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
        public PathChain Path9;

        public Paths(Follower follower) {
            // Go to shooting pos
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.000, 9.000),
                            new Pose(45.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Park
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
                // Primary case: Sit for 1 second reading tag with Pipeline 0
                follower.setMaxPower(0.6);
                limeLight.getLimelight().pipelineSwitch(0);
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    // Switch to Pipeline 2 (Blue) and start moving
                    limeLight.RelocalizationBlue();
                    turret.setTrackingTag(true);
                    follower.followPath(paths.Path1, true);
                    setPathState(1);
                }
                break;

            case 1:
                // Move to first shooting position + stabilization wait
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
                // Shooting preload balls
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.Path9, true); // Park directly after first shot
                    setPathState(15);
                }
                break;

            case 15:
                // Parking wait
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
