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

    // Poses - from original buildPaths()
    private final Pose startPose = new Pose(25, 129, Math.toRadians(143.5));
    private final Pose tagPose = new Pose(44.473, 94, Math.toRadians(60));
    private final Pose shootPose = new Pose(44.473, 94, Math.toRadians(143.5));
    private final Pose rotatedPose = new Pose(41.602, 84.473, Math.toRadians(180));
    private final Pose leftPose = new Pose(17.290, 84.430, Math.toRadians(180));
    private final Pose endPose = new Pose(24.097, 70.968, Math.toRadians(180));

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
        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null);
        shooter.Initialize(hardwareMap);

        // NOW create follower and set starting pose AFTER subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized - AlbastruScurtIncomplet");
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

        // Path 3: Rotate left (143.5° → 180°)
        rotateLeft = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose))
                .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))
                .setConstraints(straightConstraints)
                .build();

        // Path 4: Move left (180° constant)
        Path goLeftPath = new Path(new BezierLine(
                new Pose(41.602, 84.473, Math.toRadians(180)),
                new Pose(17.290, 84.430, Math.toRadians(180))
        ), straightConstraints);
        goLeftPath.setConstantHeadingInterpolation(Math.toRadians(180));
        goLeft = follower.pathBuilder()
                .addPath(goLeftPath)
                .build();

        // Path 5: Return to end position (180° → 180°)
        returnToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        leftPose,
                        endPose
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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

