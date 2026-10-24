
package org.firstinspires.ftc.teamcode.PedroAutoTurret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "RL15CazBun", group = "Autonomous")
@Configurable // Panels
public class RL15CazBun extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)
    private int humanCycles = 0;

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private LimeLight limeLight;
    private Shooter shooter;
    private Mixer mixer;
    private TurretProfiledPIDControl turret;

    private Paths paths; // Paths defined in the Paths class

    private final double maxShootingTime = 2.0; // seconds
    private final double pickupWaitTime = 0.3; // seconds

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        // Initialize LimeLight for Blue alliance
        limeLight = new LimeLight(false, true);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        shooter = new Shooter(telemetryCustom, mixer, limeLight, true);
        shooter.Initialize(hardwareMap);
        shooter.ForceUpdateShooterF();

        turret = new TurretProfiledPIDControl(limeLight, null);
        turret.Initialize(hardwareMap, true);
        turret.setUsePinpointFallback(false);
        turret.setTargetAngle(87);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144 - 57, 9, Math.toRadians(0)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        turret.Run();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.SetMotorPower(1);
        // shooter.StartBackMotorAuto();
        shooter.StartAutoBoost(); // Optimized spin-up
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
        panelsTelemetry.debug("Turret Angle", turret.getCurrentAngle());
        panelsTelemetry.debug("Shooter RPM", shooter.GetVelocityCurrent());
        panelsTelemetry.debug("Limelight ID", limeLight.GetID());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain ShootingPos;
        public PathChain FirstStackPos;
        public PathChain PickUpStack;
        public PathChain PickUpHumanBalls1;
        public PathChain GoBackHuman;
        public PathChain PickUpHumanBalls2;
        public PathChain Park;

        public Paths(Follower follower) {
            ShootingPos = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(144 - 57.000, 9.000),
                            new Pose(144 - 48.000, 11.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            PickUpHumanBalls1 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(144 - 48.000, 11.000),
                            new Pose(144 - 28.341, 9.247),
                            new Pose(144 - 11.500, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GoBackHuman = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(144 - 11.500, 12.000),
                            new Pose(144 - 28.500, 11.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            PickUpHumanBalls2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(144 - 28.500, 11.000),
                            new Pose(144 - 15.800, 9.000),
                            new Pose(144 - 10.000, 9.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Park = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(144 - 48.000, 11.000),
                            new Pose(115, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            FirstStackPos = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(144 - 48.000, 11.000),

                            new Pose(144 - 41.745, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            PickUpStack = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(144 - 41.745, 36.000),

                            new Pose(144 - 11.211, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to ShootingPos
                follower.setMaxPower(0.8);
                follower.followPath(paths.ShootingPos, true);
                limeLight.getLimelight().pipelineSwitch(0);
                // turret.setTrackingTag(false);
                setPathState(1);
                break;

            case 1: // Wait arrive
                if (!follower.isBusy())
                    setPathState(2);
                break;

            case 2: // Wait Tag
                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (limeLight.GetID() == 0)
                        limeLight.SkipArtifactDetection();
                    setPathState(4);
                }
                break;

            case 4: // Wait Turret + Back Motor
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 1.5
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 2)) {
                    // limeLight.RelocalizationRed();
                    // turret.setTrackingTag(true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5: // Settle and Start Shoot
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6: // Monitor Shot 1
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.FirstStackPos, true);
                    setPathState(7);
                }
                break;

            case 7: // Move to FirstStackPos
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickUpStack, true);
                    intake.SetPowerMax();
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8: // Picking up Stack
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.ShootingPos, true);
                        setPathState(9);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 9: // Arrive ShootingPos for Shot 2
                mixer.Run();
                if (!follower.isBusy()) {
                    // limeLight.getLimelight().pipelineSwitch(0);
                    setPathState(12);
                }
                break;

            case 12: // Wait Turret 2
                mixer.Run();
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 1.7
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 2)) {
                    // limeLight.RelocalizationRed();
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13: // Shoot 2
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14: // Monitor Shot 2
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.PickUpHumanBalls1, true);
                    setPathState(15);
                }
                break;

            case 15: // Pickup Human 1
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.GoBackHuman, true);
                    setPathState(16);
                }
                break;

            case 16: // Pickup Human 2 Curve
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickUpHumanBalls2, true);
                    setPathState(17);
                }
                break;

            case 17: // Wait Pickup Human
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.ShootingPos, true);
                        setPathState(18);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 18: // Arrive ShootingPos for Shot 3
                mixer.Run();
                if (!follower.isBusy()) {
                    // limeLight.getLimelight().pipelineSwitch(0);
                    setPathState(21);
                }
                break;

            case 21: // Wait Turret 3
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 1.7
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 2)) {
                    // limeLight.RelocalizationBlue();
                    pathTimer.resetTimer();
                    setPathState(22);
                }
                break;

            case 22: // Shoot 3
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.StartAutoShoot();
                    setPathState(23);
                }
                break;

            case 23: // Monitor Shot 3
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    humanCycles++;
                    if (humanCycles < 3) {
                        follower.followPath(paths.PickUpHumanBalls1, true);
                        setPathState(15);
                    } else {
                        follower.followPath(paths.Park, true);
                        setPathState(24);
                    }
                }
                break;

            case 24: // Final Park
                if (!follower.isBusy()) {
                    intake.SetMotorPower(0);
                    setPathState(-1);
                }
                break;
        }
    }
}
