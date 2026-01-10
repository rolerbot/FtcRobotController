package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "RosuLung", group = "Pedro Pathing")
public class RosuLung extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private Mixer mixer;

    // Poses - MIRRORED from AlbastruLung (X_red = 144 - X_blue, heading mirrored)
    private final Pose startPose = new Pose(88, 9, Math.toRadians(90));           // 144-56=88
    private final Pose tagPose = new Pose(88, 39, Math.toRadians(90));            // Same Y, same heading
    private final Pose shootPose = new Pose(88, 17, Math.toRadians(68));          // 180-112=68 - Y=17
    private final Pose rotatedPose = new Pose(92, 45, Math.toRadians(0));       // 144-52=92, 180-180=0
    private final Pose rightPose = new Pose(120, 45, Math.toRadians(0));        // 144-20=124
    private final Pose intermediatePose = new Pose(109, 30, Math.toRadians(-15)); // 144-35=109, -(-164.5)=164.5
    private final Pose humanPlayerPose = new Pose(119, 17, Math.toRadians(-15));  // 144-25=119
    private final Pose endPose = new Pose(88, 35, Math.toRadians(68));            // Same as shootPose heading
    private final double waitTime = 4.5;
    private final double pickupWaitTime = 2.25;

    // Paths - MATCH AlbastruLung structure exactly
    private Path toTag;
    private PathChain toShoot1, rotateRight, goRight, returnToShoot, toIntermediate, toHumanPlayer, returnFromHuman, toEnd;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        husky = new Husky(telemetryCustom);
        husky.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts();

        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null);
        shooter.Initialize(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Position", "X=88, Y=9, H=90°");
        telemetry.addData("Mixer", "3 balls loaded");
        telemetry.update();
    }

    public void buildPaths() {
        PathConstraints normalConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);
        PathConstraints slowConstraints = new PathConstraints(0.3, 30, 0.4, 0.4);

        // Path 0: Forward to TAG
        toTag = new Path(new BezierLine(startPose, tagPose));
        toTag.setConstantHeadingInterpolation(Math.toRadians(90));

        // Path 1: Back to shooting position (Y=39 to Y=20)
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(tagPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();

        // Path 2: Rotation move (Y=20 to Y=37)
        rotateRight = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();

        // Path 3: Straight right movement - SLOW (Y=37)
        Path goRightPath = new Path(new BezierLine(rotatedPose, rightPose), slowConstraints);
        goRightPath.setConstantHeadingInterpolation(Math.toRadians(0));
        goRight = follower.pathBuilder()
                .addPath(goRightPath)
                .build();

        // Path 4: Return to shoot position - updated control points for Y=20
        returnToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        rightPose,
                        new Pose(114, 28),   // Adjusted Y for new shootPose
                        new Pose(99, 24),    // Adjusted Y for new shootPose
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();

        // Path 5: Go to intermediate position (heading now -15°)
        toIntermediate = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intermediatePose))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(-15))
                .build();

        // Path 6: Go to human player position - VERY SLOW (heading -15°)
        PathConstraints verySlowConstraints = new PathConstraints(0.15, 20, 0.3, 0.3);
        Path toHumanPlayerPath = new Path(new BezierLine(intermediatePose, humanPlayerPose), verySlowConstraints);
        toHumanPlayerPath.setConstantHeadingInterpolation(Math.toRadians(-15));
        toHumanPlayer = follower.pathBuilder()
                .addPath(toHumanPlayerPath)
                .build();

        // Path 7: Return from human player to shoot (heading -15° to 68°)
        returnFromHuman = follower.pathBuilder()
                .addPath(new BezierCurve(
                        humanPlayerPose,
                        new Pose(109, 22),
                        new Pose(96, 26),    // Adjusted Y for new shootPose
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(68))
                .build();

        // Path 8: Go to end position (Y=20 to Y=35)
        toEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setConstantHeadingInterpolation(Math.toRadians(68))
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

        shooter.Run();
        if (!shooter.GetShootingAllow())
            mixer.Run();
        husky.Run();

        autonomousPathUpdate();

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
                    setPathState(3);
                }
                break;

            case 3:
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();
                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 2.0) {
                    shooter.StartAutoShoot();
                    setPathState(4);
                }
                break;

            case 4:
                if (shooter.AutoShoot()) {
                    follower.followPath(rotateRight, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    Pose currentPos = follower.getPose();
                    telemetry.addData("🔍 Case 5", "Starting goRight");
                    telemetry.addData("Current", "X=%.1f, Y=%.1f", currentPos.getX(), currentPos.getY());
                    telemetry.addData("Expected", "X=%.0f, Y=%.0f", rotatedPose.getX(), rotatedPose.getY());

                    intake.SetPowerMax();
                    follower.setMaxPower(0.25);
                    follower.followPath(goRight, true);
                    setPathState(6);
                }
                break;

            case 6:
                follower.setMaxPower(0.25);
                // Intake running, mixer collecting balls

                // DIAGNOSTIC: Show why we're stuck
                Pose currentPos = follower.getPose();
                double distToRight = Math.sqrt(
                    Math.pow(currentPos.getX() - rightPose.getX(), 2) +
                    Math.pow(currentPos.getY() - rightPose.getY(), 2)
                );
                telemetry.addData("🔍 Case 6", "Going right");
                telemetry.addData("Dist to target", "%.1f in", distToRight);
                telemetry.addData("Target", "X=%.0f, Y=%.0f", rightPose.getX(), rightPose.getY());

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(returnToShoot, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    shooter.StartAutoShoot();
                    setPathState(8);
                }
                break;

            case 8:
                if (shooter.AutoShoot()) {
                    follower.followPath(toIntermediate, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.2);
                    follower.followPath(toHumanPlayer, true);
                    setPathState(10);
                }
                break;

            case 10:
                follower.setMaxPower(0.2);
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                    follower.followPath(returnFromHuman, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    shooter.StartAutoShoot();
                    setPathState(13);
                }
                break;

            case 13:
                if (shooter.AutoShoot()) {
                    intake.SetMotorPower(0.0);
                    follower.followPath(toEnd, true);
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
