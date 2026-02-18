package org.firstinspires.ftc.teamcode.pedroPathing.TestAuto;

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

@Autonomous(name = "AlbastruScurt9BallShortParkTest", group = "Autonomous")
@Configurable
public class AlbastruScurt9BallShortParkTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private LimeLight limeLight;
    private Shooter shooter;
    private Mixer mixer;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        limeLight = new LimeLight(true, false);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts();

        shooter = new Shooter(telemetryCustom, mixer, limeLight, false);
        shooter.Initialize(hardwareMap);
        //shooter.ForceUpdateShooterF();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.092, 120.829, Math.toRadians(144)));

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Mixer", "3 balls loaded");
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths() {
        // Path 1: First curve to intermediate
        Path1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(19.092, 120.829),
                        new Pose(61, 130),
                        new Pose(56, 107.139)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(90))
                .build();

        // Path 2: Line to shooting position
        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(56, 107.139),
                        new Pose(53.000, 92.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(136))
                .build();

        // Path 3: Move to first sample (4th ball)
        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(53, 92),
                        new Pose(44.500, 117.500)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(90))
                .build();

        // Path 4: Back to shooting position
        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(44.500, 117.500),
                        new Pose(53, 92)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(136))
                .build();

        // Path 5: Move to second sample (5th ball)
        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(53, 92),
                        new Pose(54.500, 117.500)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(90))
                .build();

        // Path 6: Back to shooting position
        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(54.500, 117.500),
                        new Pose(53, 92)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(136))
                .build();

        // Path 7: Move to third sample (6th ball)
        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(53, 92),
                        new Pose(64.500, 117.500)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(90))
                .build();

        // Path 8: Park
        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(64.500, 117.500),
                        new Pose(17, 107)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
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

        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooter RPM", shooter.GetVelocityCurrent());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3:
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 1.7) {
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

            case 7:
                currentRPM = shooter.GetVelocityCurrent();
                targetRPM = shooter.GetVelocityTarget();

                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 1.7) {
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(Path5, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path6, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11:
                currentRPM = shooter.GetVelocityCurrent();
                targetRPM = shooter.GetVelocityTarget();

                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 1.7) {
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                if (shooter.AutoShoot() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    follower.followPath(Path7, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(Path8, true);
                    intake.SetMotorPower(0.0);
                    setPathState(14);
                }
                break;

            case 14:
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
