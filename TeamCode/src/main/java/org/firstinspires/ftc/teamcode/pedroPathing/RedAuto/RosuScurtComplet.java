package org.firstinspires.ftc.teamcode.pedroPathing.RedAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "RosuScurtComplet", group = "Pedro Pathing")
public class RosuScurtComplet extends OpMode {
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
    private final Pose rotatedPose1 = new Pose(102.398, 84.473, Math.toRadians(0));
    private final Pose rightPose1 = new Pose(126.710, 84.430, Math.toRadians(0));
    private final Pose rotatedPose2 = new Pose(100.419, 60.753, Math.toRadians(0));
    private final Pose rightPose2 = new Pose(126.710, 60.054, Math.toRadians(0));
    private final Pose rotatedPose3 = new Pose(100.280, 35.656, Math.toRadians(0));
    private final Pose rightPose3 = new Pose(127.624, 35.344, Math.toRadians(0));
    private final Pose endPose = new Pose(87, 15, Math.toRadians(68));

    // Paths
    private Path toTag;
    private PathChain toShoot1, rotateRight1, goRight1, returnToShoot1;
    private PathChain rotateRight2, goRight2, returnToShoot2;
    private PathChain rotateRight3, goRight3, returnToEnd;

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

        telemetry.addData("Status", "Initialized - RosuScurtComplet");
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

        // First pickup cycle
        rotateRight1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose1))
                .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                .setConstraints(straightConstraints)
                .build();

        Path goRight1Path = new Path(new BezierLine(rotatedPose1, rightPose1), straightConstraints);
        goRight1Path.setConstantHeadingInterpolation(Math.toRadians(0));
        goRight1 = follower.pathBuilder()
                .addPath(goRight1Path)
                .build();

        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(rightPose1, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5))
                .setConstraints(straightConstraints)
                .build();

        // Second pickup cycle
        rotateRight2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose2))
                .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                .setConstraints(straightConstraints)
                .build();

        Path goRight2Path = new Path(new BezierLine(rotatedPose2, rightPose2), straightConstraints);
        goRight2Path.setConstantHeadingInterpolation(Math.toRadians(0));
        goRight2 = follower.pathBuilder()
                .addPath(goRight2Path)
                .build();

        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(rightPose2, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5))
                .setConstraints(straightConstraints)
                .build();

        // Third pickup cycle and end
        rotateRight3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose3))
                .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                .setConstraints(straightConstraints)
                .build();

        Path goRight3Path = new Path(new BezierLine(rotatedPose3, rightPose3), straightConstraints);
        goRight3Path.setConstantHeadingInterpolation(Math.toRadians(0));
        goRight3 = follower.pathBuilder()
                .addPath(goRight3Path)
                .build();

        returnToEnd = follower.pathBuilder()
                .addPath(new BezierCurve(
                        rightPose3,
                        new Pose(113, 28),
                        new Pose(100, 21),
                        endPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .setConstraints(rotationConstraints)
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
                    follower.followPath(rotateRight1, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goRight1, true);
                    setPathState(6);
                }
                break;

            case 6:
                follower.setMaxPower(0.25);
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(returnToShoot1, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    telemetry.addData("✅", "Back at shoot - starting auto shoot");
                    shooter.StartAutoShoot();
                    setPathState(8);
                }
                break;

            case 8:
                if (shooter.AutoShoot()) {
                    telemetry.addData("✅", "Done second shooting!");
                    follower.followPath(rotateRight2, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goRight2, true);
                    setPathState(10);
                }
                break;

            case 10:
                follower.setMaxPower(0.25);
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(returnToShoot2, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    telemetry.addData("✅", "Back - third shooting");
                    shooter.StartAutoShoot();
                    setPathState(12);
                }
                break;

            case 12:
                if (shooter.AutoShoot()) {
                    telemetry.addData("✅", "Done third shooting!");
                    follower.followPath(rotateRight3, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goRight3, true);
                    setPathState(14);
                }
                break;

            case 14:
                follower.setMaxPower(0.25);
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    intake.SetMotorPower(0.0); // Stop intake
                    follower.followPath(returnToEnd, true);
                    setPathState(15);
                }
                break;

            case 15:
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

