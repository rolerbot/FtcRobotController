package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "AlbastruScurtComplet", group = "Pedro Pathing")
public class AlbastruScurtComplet extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private Mixer mixer;

    // Poses - from original buildPaths()
    private final Pose startPose = new Pose(25, 129, Math.toRadians(143.5));
    private final Pose tagPose = new Pose(44.473, 94, Math.toRadians(60));
    private final Pose shootPose = new Pose(44.473, 94, Math.toRadians(143.5));
    private final Pose rotatedPose1 = new Pose(41.602, 84.473, Math.toRadians(180));
    private final Pose leftPose1 = new Pose(17.290, 84.430, Math.toRadians(180));
    private final Pose rotatedPose2 = new Pose(43.581, 60.753, Math.toRadians(180));
    private final Pose leftPose2 = new Pose(17.290, 60.054, Math.toRadians(180));
    private final Pose rotatedPose3 = new Pose(43.720, 35.656, Math.toRadians(180));
    private final Pose leftPose3 = new Pose(16.376, 35.344, Math.toRadians(180));
    private final Pose endPose = new Pose(56, 15, Math.toRadians(112));

    // Paths
    private Path toTag;
    private PathChain toShoot1, rotateLeft1, goLeft1, returnToShoot1;
    private PathChain rotateLeft2, goLeft2, returnToShoot2;
    private PathChain rotateLeft3, goLeft3, returnToEnd;

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

        telemetry.addData("Status", "Initialized - AlbastruScurtComplet");
        telemetry.addData("Start Position", "X=25, Y=129, H=143.5°");
        telemetry.addData("Mixer", "3 balls loaded");
        telemetry.update();
    }

    public void buildPaths() {
        final double ROTATION_COMPENSATION = 6.0;
        PathConstraints rotationConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);
        PathConstraints straightConstraints = new PathConstraints(0.3, 30, 0.5, 0.5);

        // Path 1: Start to tag (143.5° → 60°)
        toTag = new Path(new BezierCurve(
                new Pose(25, 129),
                new Pose(32, 120),
                new Pose(38, 110),
                new Pose(44.473, 100.172 - ROTATION_COMPENSATION)
        ), rotationConstraints);
        toTag.setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(60));

        // Path 2: Rotate back and wait at shoot position (60° → 143.5°)
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        tagPose,
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(143.5))
                .build();

        // First pickup cycle
        rotateLeft1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose1))
                .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))
                .setConstraints(straightConstraints)
                .build();

        Path goLeft1Path = new Path(new BezierLine(rotatedPose1, leftPose1), straightConstraints);
        goLeft1Path.setConstantHeadingInterpolation(Math.toRadians(180));
        goLeft1 = follower.pathBuilder()
                .addPath(goLeft1Path)
                .build();

        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(leftPose1, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5))
                .setConstraints(straightConstraints)
                .build();

        // Second pickup cycle
        rotateLeft2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose2))
                .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))
                .setConstraints(straightConstraints)
                .build();

        Path goLeft2Path = new Path(new BezierLine(rotatedPose2, leftPose2), straightConstraints);
        goLeft2Path.setConstantHeadingInterpolation(Math.toRadians(180));
        goLeft2 = follower.pathBuilder()
                .addPath(goLeft2Path)
                .build();

        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(leftPose2, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143.5))
                .setConstraints(straightConstraints)
                .build();

        // Third pickup cycle and end
        rotateLeft3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose3))
                .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))
                .setConstraints(straightConstraints)
                .build();

        Path goLeft3Path = new Path(new BezierLine(rotatedPose3, leftPose3), straightConstraints);
        goLeft3Path.setConstantHeadingInterpolation(Math.toRadians(180));
        goLeft3 = follower.pathBuilder()
                .addPath(goLeft3Path)
                .build();

        returnToEnd = follower.pathBuilder()
                .addPath(new BezierCurve(
                        leftPose3,
                        new Pose(30, 28),
                        new Pose(43, 21),
                        endPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
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
                    follower.followPath(rotateLeft1, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goLeft1, true);
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
                    follower.followPath(rotateLeft2, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goLeft2, true);
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
                    follower.followPath(rotateLeft3, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goLeft3, true);
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

