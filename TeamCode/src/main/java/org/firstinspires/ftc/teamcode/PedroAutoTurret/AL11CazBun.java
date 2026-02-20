
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

@Autonomous(name = "AL11CazBun", group = "Autonomous")
@Configurable // Panels
public class AL11CazBun extends OpMode {
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

        private final double maxShootingTime = 3; // seconds (Updated to 2.2)
        private final double pickupWaitTime = 0.5; // seconds

        @Override
        public void init() {
                panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
                pathTimer = new Timer();
                opmodeTimer = new Timer();

                telemetryCustom = new TelemetryCustom(telemetry);

                intake = new Intake(telemetryCustom);
                intake.Initialize(hardwareMap);

                // Initialize LimeLight for Blue alliance (Pipeline 2 requested for Blue)
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
                turret.setUsePinpointFallback(false); // ✅ Disable odometry fallback in Auto
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
                // Mixer called broadly as requested

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
                public PathChain Path5Extra;
                public PathChain Path6;
                public PathChain Path7;
                public PathChain Path7Extra;
                public PathChain Path8;
                public PathChain Path9;

                public Paths(Follower follower) {
                        // Go to shooting pos
                        Path1 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(56.000, 9.000),
                                                        new Pose(45.000, 17.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // Prepare to pick up Balls
                        Path2 = follower.pathBuilder().addPath(
                                        new BezierCurve(
                                                        new Pose(45.000, 17.000),
                                                        new Pose(60.618, 35.440),
                                                        new Pose(46.478, 35.594)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // Pick up balls
                        Path3 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(46.478, 35.594),
                                                        new Pose(13, 35.869)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // go to shooting pose
                        Path4 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(13, 35.869),
                                                        new Pose(45.000, 17.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // go to human player
                        Path5 = follower.pathBuilder().addPath(
                                        new BezierCurve(
                                                        new Pose(45.000, 17.000),
                                                        new Pose(35.076, 12.950),
                                                        new Pose(10.000, 12.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // Path 5 Extra: slow crawl 1 inch further at HP
                        Path5Extra = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(10.000, 12.000),
                                                        new Pose(7.000, 12.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // go to shoting pose
                        Path6 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(7.000, 12.000),
                                                        new Pose(45.000, 17.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // go to humanplayer closer
                        Path7 = follower.pathBuilder().addPath(
                                        new BezierCurve(
                                                        new Pose(45.000, 17.000),
                                                        new Pose(39.147, 12.484),
                                                        new Pose(10.000, 12.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // Path 7 Extra: slow crawl 1 inch further at HP
                        Path7Extra = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(10.000, 12.000),
                                                        new Pose(7.000, 12.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // go to shooting pose
                        Path8 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(7.000, 12.000),
                                                        new Pose(45.000, 17.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                        .build();

                        // Park
                        Path9 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(54.000, 17.000),
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
                                        turret.setTrackingTag(true); // Fixed on tag logic
                                        follower.followPath(paths.Path1, true);
                                        setPathState(1);
                                }
                                break;

                        case 1:
                                // Move to first shooting position + 0.5s stabilizer
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
                                // Shooting first setup
                                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                                        follower.followPath(paths.Path2, true);
                                        setPathState(3);
                                }
                                break;

                        case 3:
                                // Prepare to pick up balls
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        intake.SetPowerMax();
                                        follower.followPath(paths.Path3, true);
                                        follower.setMaxPower(0.5);
                                        setPathState(4);
                                }
                                break;

                        case 4:
                                // Picking up balls
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                                                follower.followPath(paths.Path4, true);
                                                follower.setMaxPower(0.8);
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
                                        if (pathTimer.getElapsedTimeSeconds() > 1.3) {
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
                                        follower.setMaxPower(0.6);
                                        setPathState(7);
                                }
                                break;

                        case 7:
                                // Reach Human Player, then Wait
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        intake.SetPowerMax();
                                        if (pathTimer.getElapsedTimeSeconds() > 1.0) { // HP Wait time
                                                follower.setMaxPower(0.4); // Slow nudge forward
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
                                        follower.setMaxPower(0.8);
                                        follower.followPath(paths.Path6, true);
                                        follower.setMaxPower(0.7);
                                        setPathState(9);
                                }
                                break;

                        case 9:
                                // Stabilize and Shoot
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        if (pathTimer.getElapsedTimeSeconds() > 1.3) {
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
                                        follower.setMaxPower(0.6);
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
                                                follower.setMaxPower(0.4); // Slow nudge forward
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
                                        follower.setMaxPower(0.7);
                                        follower.followPath(paths.Path8, true);
                                        setPathState(13);
                                }
                                break;

                        case 13:
                                // Stabilize and Shoot
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                                                shooter.StartAutoShoot();
                                                pathTimer.resetTimer();
                                                setPathState(14);
                                        }
                                } else {
                                        pathTimer.resetTimer();
                                }
                                break;

                        case 14:
                                // Finish Shooting, then Park
                                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                                        follower.followPath(paths.Path9, true);
                                        follower.setMaxPower(0.8);
                                        setPathState(15);
                                }
                                break;

                        case 15:
                                // Parking
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
