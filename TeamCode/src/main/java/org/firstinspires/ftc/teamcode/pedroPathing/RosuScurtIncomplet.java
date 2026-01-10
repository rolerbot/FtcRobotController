package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "RosuScurtIncomplet", group = "Pedro Pathing")
public class RosuScurtIncomplet extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private Mixer mixer;

    // Poses - from original buildPaths() (mirrored from blue)
    private final Pose startPose = new Pose(119, 129, Math.toRadians(36.5));
    private final Pose tagPose = new Pose(99.527, 94, Math.toRadians(120));
    private final Pose shootPose = new Pose(99.527, 94, Math.toRadians(36.5));
    private final Pose rotatedPose = new Pose(102.398, 84.473, Math.toRadians(0));
    private final Pose rightPose = new Pose(126.710, 84.430, Math.toRadians(0));
    private final Pose endPose = new Pose(119.903, 70.968, Math.toRadians(0));

    // Paths
    private Path toTag;
    private PathChain toShoot1, rotateRight, goRight, returnToEnd;

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
        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null);
        shooter.Initialize(hardwareMap);

        // NOW create follower and set starting pose AFTER subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized - RosuScurtIncomplet");
        telemetry.addData("Start Position", "X=119, Y=129, H=36.5°");
        telemetry.addData("Mixer", "3 balls loaded");
        telemetry.update();
    }

    public void buildPaths() {
        final double ROTATION_COMPENSATION = 6.0;
        PathConstraints rotationConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);
        PathConstraints straightConstraints = new PathConstraints(0.3, 30, 0.5, 0.5);

        // Path 1: Start to tag (36.5° → 120°)
        toTag = new Path(new BezierCurve(
                new Pose(119, 129),
                new Pose(111, 120),
                new Pose(105, 110),
                new Pose(99.527, 100.172 - ROTATION_COMPENSATION)
        ), rotationConstraints);
        toTag.setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(120));

        // Path 2: Rotate back and wait at shoot position (120° → 36.5°)
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        tagPose,
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(36.5))
                .build();

        // Path 3: Rotate right (36.5° → 0°)
        rotateRight = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose))
                .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                .setConstraints(straightConstraints)
                .build();

        // Path 4: Move right (0° constant)
        Path goRightPath = new Path(new BezierLine(
                new Pose(102.398, 84.473, Math.toRadians(0)),
                new Pose(126.710, 84.430, Math.toRadians(0))
        ), straightConstraints);
        goRightPath.setConstantHeadingInterpolation(Math.toRadians(0));
        goRight = follower.pathBuilder()
                .addPath(goRightPath)
                .build();

        // Path 5: Return to end position (0° constant)
        returnToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        rightPose,
                        endPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
                    follower.followPath(rotateRight, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goRight, true);
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

