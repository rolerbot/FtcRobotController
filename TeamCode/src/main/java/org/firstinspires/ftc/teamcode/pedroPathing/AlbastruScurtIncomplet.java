package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "AlbastruScurtIncomplet", group = "Pedro Pathing")
public class AlbastruScurtIncomplet extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private Mixer mixer;

    // Poses - Shifted to start from (0, 0, 90°)
    // Original start: (25, 129, 143.5°)
    // Transformation: new_x = old_x - 25, new_y = old_y - 129, new_heading = old_heading - 53.5°
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose tagPose = new Pose(19.473, -35, Math.toRadians(6.5));
    private final Pose shootPose = new Pose(19.473, -35, Math.toRadians(90));
    private final Pose rotatedPose = new Pose(16.602, -44.527, Math.toRadians(126.5));
    private final Pose leftPose = new Pose(-7.710, -44.570, Math.toRadians(126.5));
    private final Pose endPose = new Pose(-0.903, -58.032, Math.toRadians(126.5));

    // Paths
    private Path toTag;
    private PathChain toShoot1, rotateLeft, goLeft, returnToEnd;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // CRITICAL: Initialize subsystems FIRST, before creating follower
        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        husky = new Husky(telemetryCustom);
        husky.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        // Pass null for robotAllignment - no IMU conflicts!
        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null, false);
        shooter.Initialize(hardwareMap);

        // NOW create follower and set starting pose AFTER subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized - AlbastruScurtIncomplet");
        telemetry.addData("Start Position", "X=0, Y=0, H=90°");
        telemetry.addData("Mixer", "3 balls loaded");
        telemetry.update();
    }

    public void buildPaths() {
        final double ROTATION_COMPENSATION = 6.0;
        PathConstraints rotationConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);
        PathConstraints straightConstraints = new PathConstraints(0.3, 30, 0.5, 0.5);

        // Path 1: Start to tag (90° → 6.5°)
        toTag = new Path(new BezierCurve(
                new Pose(0, 0),
                new Pose(13, -19),
                new Pose(19.473, -28.828 - ROTATION_COMPENSATION)
        ), rotationConstraints);
        toTag.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(6.5));

        // Path 2: Rotate back and wait at shoot position (6.5° → 90°)
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        tagPose,
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(6.5), Math.toRadians(90))
                .build();

        // Path 3: Rotate left (90° → 126.5°)
        rotateLeft = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(126.5))
                .setConstraints(straightConstraints)
                .build();

        // Path 4: Move left (126.5° constant)
        Path goLeftPath = new Path(new BezierLine(
                new Pose(16.602, -44.527, Math.toRadians(126.5)),
                new Pose(-7.710, -44.570, Math.toRadians(126.5))
        ), straightConstraints);
        goLeftPath.setConstantHeadingInterpolation(Math.toRadians(126.5));
        goLeft = follower.pathBuilder()
                .addPath(goLeftPath)
                .build();

        // Path 5: Return to end position (126.5° constant)
        returnToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        leftPose,
                        endPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(126.5))
                .setConstraints(straightConstraints)
                .build();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.SetMotorPower(0.8);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // Update subsystems
        shooter.Run();
        if (!shooter.GetShootingAllow())
            mixer.Run();
        husky.Run();

        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("shooter rpm", shooter.GetVelocityCurrent());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(toTag);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(toShoot1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    telemetry.addData("✅", "At shoot position - shooter spinning");
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                telemetry.addData("Shooter", "%.0f / %.0f RPM", currentRPM, targetRPM);

                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 2.0) {
                    telemetry.addData("✅", "Shooting!");
                    shooter.StartAutoShoot();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for shooting to complete
                if (shooter.AutoShoot()) {
                    telemetry.addData("✅", "Done shooting!");
                    follower.followPath(rotateLeft, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goLeft, true);
                    setPathState(6);
                }
                break;

            case 6:
                follower.setMaxPower(0.25);
                // Intake running, collecting balls

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    intake.SetMotorPower(0.0); // Stop intake
                    follower.followPath(returnToEnd, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Park
                if (!follower.isBusy()) {
                    telemetry.addData("✅✅✅", "COMPLETE!");
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