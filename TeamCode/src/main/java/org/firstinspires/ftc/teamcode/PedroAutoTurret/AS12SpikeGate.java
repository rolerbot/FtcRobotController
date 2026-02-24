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

@Autonomous(name = "AS12SpikeGate", group = "Autonomous")
@Configurable // Panels
public class AS12SpikeGate extends OpMode {
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

        private final double maxShootingTime = 2.8; // seconds
        private final double pickupWaitTime = 0.5; // seconds

        @Override
        public void init() {
                panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
                pathTimer = new Timer();
                opmodeTimer = new Timer();

                telemetryCustom = new TelemetryCustom(telemetry);

                intake = new Intake(telemetryCustom);
                intake.Initialize(hardwareMap);

                // Initialize LimeLight for Blue alliance (Pipeline 0 for detection first)
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
                turret.setUsePinpointFallback(false); // Disable odometry fallback
                turret.setTargetTicks(0); // Set turret to encoder position 0 (physical start) in init

                mixer.Reset(); // Ensure mixer is at home and encoder is zero

                follower = Constants.createFollower(hardwareMap);
                follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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
                panelsTelemetry.debug("Limelight ID", limeLight.GetID());
                panelsTelemetry.update(telemetry);
        }

        public static class Paths {
                public PathChain Path1;
                public PathChain Path2;
                public PathChain Path3;
                public PathChain Path5;
                public PathChain Path6;
                public PathChain Path7;
                public PathChain Path13;
                public PathChain Path15;
                public PathChain Path14;
                public PathChain Path8;
                public PathChain Path9;
                public PathChain Path10;
                public PathChain Path11;
                public PathChain Path12;

                public Paths(Follower follower) {

                        Path1 = follower.pathBuilder().addPath(
                                        new BezierCurve(
                                                        new Pose(33.476, 134.481),
                                                        new Pose(57.369, 124.144),
                                                        new Pose(59.080, 112.235)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                                        .build();

                        Path2 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(59.080, 112.235),

                                                        new Pose(54.000, 85.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                                        .build();

                        Path3 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(54.000, 85.000),

                                                        new Pose(15.610, 84.979)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();

                        Path5 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(15.610, 84.979),

                                                        new Pose(54.000, 85.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();

                        Path6 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(54.000, 85.000),

                                                        new Pose(53.225, 59.615)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();

                        Path7 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(53.225, 59.615),

                                                        new Pose(16.000, 59.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();

                        Path13 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(16.000, 59.000),

                                                        new Pose(35.000, 59.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();

                        Path14 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(35.000, 59.000),

                                                        new Pose(20.000, 70.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                                        .build();

                        Path15 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(20.000, 70.000),

                                                        new Pose(17.000, 70.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                                        .build();

                        Path8 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(17.000, 70.000),

                                                        new Pose(55.000, 85.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                                        .build();

                        Path9 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(55.000, 85.000),

                                                        new Pose(49.765, 35.610)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();

                        Path10 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(49.765, 35.610),

                                                        new Pose(13.636, 35.471)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();

                        Path11 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(13.636, 35.471),

                                                        new Pose(55.000, 85.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();

                        Path12 = follower.pathBuilder().addPath(
                                        new BezierLine(
                                                        new Pose(55.000, 85.000),

                                                        new Pose(30.000, 60.000)))
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                                        .build();
                }
        }

        public void autonomousPathUpdate() {
                switch (pathState) {
                        case 0:
                                // Drive to Spike position and Search (Pipeline 0)
                                follower.setMaxPower(1);
                                follower.followPath(paths.Path1, true);
                                limeLight.getLimelight().pipelineSwitch(0);
                                turret.setTrackingTag(false);
                                setPathState(1);
                                break;

                        case 1:
                                if (!follower.isBusy()) {
                                        follower.followPath(paths.Path2, true);
                                        setPathState(2);
                                }
                                break;

                        case 2:
                                if (!follower.isBusy()) {
                                        setPathState(3);
                                }
                                break;

                        case 3:
                                if (limeLight.GetID() != 0) {
                                        setPathState(4);
                                } else if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                                        setPathState(4);
                                }
                                break;

                        case 4:
                                turret.setTrackingTag(false);
                                turret.setTargetAngle(-45); // Stabilize turret
                                setPathState(5);
                                break;

                        case 5:
                                if (pathTimer.getElapsedTimeSeconds() > 0.4
                                                && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 1)) {
                                        limeLight.RelocalizationBlue();
                                        turret.setTrackingTag(true);
                                        setPathState(6);
                                }
                                break;

                        case 6:
                                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                                        shooter.StartAutoShoot();
                                        setPathState(7);
                                }
                                break;

                        case 7:
                                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                                        follower.followPath(paths.Path3, true);
                                        follower.setMaxPower(0.7);
                                        setPathState(100);
                                }
                                break;

                        case 100:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        intake.SetPowerMax();
                                        if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                                                follower.followPath(paths.Path5, true);
                                                setPathState(10);
                                        }
                                } else {
                                        pathTimer.resetTimer();
                                }
                                break;

                        case 10:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        if (pathTimer.getElapsedTimeSeconds() > 1) {
                                                shooter.StartAutoShoot();
                                                setPathState(11);
                                        }
                                } else {
                                        pathTimer.resetTimer();
                                }
                                break;

                        case 11:
                                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                                        follower.followPath(paths.Path6, true);
                                        setPathState(12);
                                }
                                break;

                        case 12:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        intake.SetPowerMax();
                                        follower.followPath(paths.Path7, true);
                                        setPathState(13);
                                }
                                break;

                        case 13:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                                                follower.followPath(paths.Path13, true);
                                                setPathState(14);
                                        }
                                } else {
                                        pathTimer.resetTimer();
                                }
                                break;

                        case 14:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        follower.followPath(paths.Path14, true);
                                        setPathState(15);
                                }
                                break;

                        case 15:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        follower.followPath(paths.Path15, true);
                                        setPathState(16);
                                }
                                break;

                        case 16:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        follower.followPath(paths.Path8, true);
                                        setPathState(17);
                                }
                                break;

                        case 17:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        if (pathTimer.getElapsedTimeSeconds() > 1) {
                                                shooter.StartAutoShoot();
                                                setPathState(18);
                                        }
                                } else {
                                        pathTimer.resetTimer();
                                }
                                break;

                        case 18:
                                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                                        follower.followPath(paths.Path9, true);
                                        setPathState(19);
                                }
                                break;

                        case 19:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        intake.SetPowerMax();
                                        follower.followPath(paths.Path10, true);
                                        setPathState(20);
                                }
                                break;

                        case 20:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                                                follower.followPath(paths.Path11, true);
                                                setPathState(21);
                                        }
                                } else {
                                        pathTimer.resetTimer();
                                }
                                break;

                        case 21:
                                mixer.Run();
                                if (!follower.isBusy()) {
                                        if (pathTimer.getElapsedTimeSeconds() > 1) {
                                                shooter.StartAutoShoot();
                                                setPathState(22);
                                        }
                                } else {
                                        pathTimer.resetTimer();
                                }
                                break;

                        case 22:
                                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                                        follower.followPath(paths.Path12, true);
                                        setPathState(23);
                                }
                                break;

                        case 23:
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
