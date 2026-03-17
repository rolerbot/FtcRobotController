
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

@Autonomous(name = "AS15CazBun", group = "Autonomous")
@Configurable // Panels
public class AS15CazBun extends OpMode {
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
    private TurretProfiledPIDControl turret;

    private Paths paths; // Paths defined in the Paths class

    private final double maxShootingTime = 1.3; // Increased from 1.0 to ensure 3 balls finish
    private final double pickupWaitTime = 1.0; // seconds

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

        turret = new TurretProfiledPIDControl(limeLight, null);
        turret.Initialize(hardwareMap);
        turret.setUsePinpointFallback(false);
        turret.setTargetAngle(0);

        mixer.MoveToThreeBalls();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(26.8, 131.5, Math.toRadians(233)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.SetMotorPower(1);
        shooter.StartAutoBoost(); // Optimized spin-up
        shooter.StartAutoBoost();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.Run();
        limeLight.Run();
        turret.Run();
        // mixer.Run(); // User requested to call this manually inside cases

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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to Shot 1
                mixer.Run();
                follower.setMaxPower(0.6);
                follower.followPath(paths.Path1, true);
                limeLight.getLimelight().pipelineSwitch(0);
                turret.setTrackingTag(false);
                setPathState(1);
                break;

            case 1: // Wait arrive Path 1
                mixer.Run();
                if (!follower.isBusy())
                    setPathState(2);
                break;

            case 2: // Pattern Step 1: Wait Tag
                mixer.Run();
                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 0.6)
                    setPathState(3);
                break;
            case 3: // Pattern Step 2: Turret pre-position
                mixer.Run();
                turret.setTrackingTag(false);
                turret.setTargetAngle(-45);
                setPathState(4);
                break;
            case 4: // Pattern Step 3: Wait turret + Relocalize
                mixer.Run();
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 0.7
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 1.0)) {
                    limeLight.RelocalizationBlue();
                    turret.setTrackingTag(true);
                    setPathState(5);
                }
                break;
            case 5: // Pattern Step 4: Settle and Shoot
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.StartAutoShoot();
                    setPathState(6);
                }
                break;
            case 6: // Monitor Shot 1
                mixer.Run();
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path2, true);
                    setPathState(7);
                }
                break;

            case 7: // Curve to Pickup 1
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.Path3, true);
                    setPathState(8);
                }
                break;

            case 8: // Pickup Line 1
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path4, true);
                        setPathState(10);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 10: // Wait arrive Path 4 (Shoot 2)
                mixer.Run();
                if (!follower.isBusy()) {
                    limeLight.getLimelight().pipelineSwitch(0);
                    setPathState(11);
                }
                break;

            // --- SHOOT 2 PATTERN (RESTORED LOGIC) ---
            case 11:
                mixer.Run();
                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 0.6)
                    setPathState(12);
                break;
            case 12:
                mixer.Run();
                turret.setTrackingTag(false);
                turret.setTargetAngle(-45);
                setPathState(13);
                break;
            case 13:
                mixer.Run();
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 0.7
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 1.0)) {
                    limeLight.RelocalizationBlue();
                    turret.setTrackingTag(true);
                    setPathState(14);
                }
                break;
            case 14:
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.StartAutoShoot();
                    setPathState(15);
                }
                break;
            case 15:
                mixer.Run();
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path5, true);
                    setPathState(16);
                }
                break;

            case 16: // Curve to Pickup 2
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.Path6, true);
                    setPathState(17);
                }
                break;

            case 17: // Pickup Line 2
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path8, true);
                        setPathState(20);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 20: // Wait arrive Path 8 (Shoot 3)
                mixer.Run();
                if (!follower.isBusy()) {
                    limeLight.getLimelight().pipelineSwitch(0);
                    setPathState(21);
                }
                break;

            // --- SHOOT 3 PATTERN (RESTORED LOGIC) ---
            case 21:
                mixer.Run();
                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 0.6)
                    setPathState(22);
                break;
            case 22:
                mixer.Run();
                turret.setTrackingTag(false);
                turret.setTargetAngle(-45);
                setPathState(23);
                break;
            case 23:
                mixer.Run();
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 0.7
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 1.0)) {
                    limeLight.RelocalizationBlue();
                    turret.setTrackingTag(true);
                    setPathState(24);
                }
                break;
            case 24:
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.StartAutoShoot();
                    setPathState(25);
                }
                break;
            case 25:
                mixer.Run();
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path9, true);
                    setPathState(26);
                }
                break;

            case 26: // Curve to Pickup 3
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.Path10, true);
                    setPathState(27);
                }
                break;

            case 27: // Pickup Line 3
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path11, true);
                        setPathState(30);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 30: // Wait arrive Path 11 (Shoot 4)
                mixer.Run();
                if (!follower.isBusy()) {
                    limeLight.getLimelight().pipelineSwitch(0);
                    setPathState(31);
                }
                break;

            // --- SHOOT 4 PATTERN (RESTORED LOGIC) ---
            case 31:
                mixer.Run();
                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 0.05)
                    setPathState(32);
                break;
            case 32:
                mixer.Run();
                turret.setTrackingTag(false);
                turret.setTargetAngle(-45);
                setPathState(33);
                break;
            case 33:
                mixer.Run();
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 0.7
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 1.0)) {
                    limeLight.RelocalizationBlue();
                    turret.setTrackingTag(true);
                    setPathState(34);
                }
                break;
            case 34:
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.StartAutoShoot();
                    setPathState(35);
                }
                break;
            case 35:
                mixer.Run();
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path12, true);
                    setPathState(36);
                }
                break;

            case 36: // Pickup 4
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.Path13, true);
                    setPathState(37);
                }
                break;

            case 37: // Pickup Line 4
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path14, true);
                        setPathState(40);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 40: // Wait arrive Path 14 (Shoot 5)
                mixer.Run();
                if (!follower.isBusy()) {
                    limeLight.getLimelight().pipelineSwitch(0);
                    setPathState(41);
                }
                break;

            // --- SHOOT 5 PATTERN (RESTORED LOGIC) ---
            case 41:
                mixer.Run();
                if (limeLight.GetID() != 0 || pathTimer.getElapsedTimeSeconds() > 0.05)
                    setPathState(42);
                break;
            case 42:
                mixer.Run();
                turret.setTrackingTag(false);
                turret.setTargetAngle(-45);
                setPathState(43);
                break;
            case 43:
                mixer.Run();
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 0.7
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 1.0)) {
                    limeLight.RelocalizationBlue();
                    turret.setTrackingTag(true);
                    setPathState(44);
                }
                break;
            case 44:
                mixer.Run();
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.StartAutoShoot();
                    setPathState(45);
                }
                break;
            case 45:
                mixer.Run();
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path15, true);
                    setPathState(46);
                }
                break;

            case 46: // Park
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetMotorPower(0);
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        intake.SetMotorPower(0.0);
        if (shooter != null)
            shooter.StopShooterMotors();
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(26.8, 131.5), new Pose(54.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(233), Math.toRadians(180)).build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(54.000, 85.000), new Pose(57.741, 61.600),
                            new Pose(42.263, 59.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.263, 59.000), new Pose(16.116, 59.000)))
                    .setTangentHeadingInterpolation().build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(16.116, 59.000), new Pose(45.488, 68.825),
                            new Pose(54.000, 85.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(54.000, 85.000), new Pose(33.500, 48.000),
                            new Pose(21, 64)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175)).build();

            Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(21, 64), new Pose(18, 53)))
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(160)).build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(18, 53), new Pose(36.000, 55.000),
                            new Pose(54.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180)).build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(54.000, 85.000), new Pose(33.500, 48.000),
                            new Pose(17.000, 63.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175)).build();

            Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(17.000, 63.000), new Pose(12.500, 54.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(160)).build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(12.500, 54.000), new Pose(36.000, 55.000),
                            new Pose(54.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180)).build();

            Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(54.000, 85.000), new Pose(42.622, 84.187)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path13 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.622, 84.187), new Pose(16.964, 84.159)))
                    .setTangentHeadingInterpolation().build();

            Path14 = follower.pathBuilder().addPath(new BezierLine(new Pose(16.964, 84.159), new Pose(54.000, 85.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path15 = follower.pathBuilder().addPath(new BezierLine(new Pose(54.000, 85.000), new Pose(19.462, 89.044)))
                    .setConstantHeadingInterpolation(Math.toRadians(180)).build();
        }
    }
}
