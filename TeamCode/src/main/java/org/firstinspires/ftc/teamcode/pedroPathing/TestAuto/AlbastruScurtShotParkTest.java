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

@Autonomous(name = "AlbastruScurtShotParkTest", group = "Autonomous")
@Configurable
public class AlbastruScurtShotParkTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private LimeLight limeLight;
    private Shooter shooter;
    private Mixer mixer;

    private PathChain Path1, Path2, Path3;

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
        // Path 1: First curve to intermediate position
        Path1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(19.092, 120.829),
                        new Pose(61, 130),
                        new Pose(56, 107.139)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(90))
                .build();

        // Path 2: Second line to shooting position
        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(56, 107.139),
                        new Pose(53.000, 92.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(136))
                .build();

        // Path 3: Move to parking position
        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(53, 92),
                        new Pose(17, 107)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(90))
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
                // Path 1: First curve to intermediate
                mixer.Run();
                follower.followPath(Path1, true);
                follower.setMaxPower(0.7);
                setPathState(1);
                break;

            case 1:
                // Wait to reach intermediate position
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for return to shoot position
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    panelsTelemetry.debug("Status", "At shoot position - shooter spinning");
                    pathTimer.resetTimer();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();
                mixer.Run();
                panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f RPM", currentRPM, targetRPM));

                if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                    panelsTelemetry.debug("Status", "Shooting preload!");
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for shooting to complete (3 preload balls)
                if (mixer.IsEmpty()) {
                    panelsTelemetry.debug("Status", "Done shooting - parking");
                    intake.SetMotorPower(0.0);
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Park and finish
                mixer.Run();
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "COMPLETE - 3 BALLS SCORED!");
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
