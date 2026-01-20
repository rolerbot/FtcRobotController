package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.*;

import javax.lang.model.type.MirroredTypeException;

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
    private final Pose startPose = new Pose(86, 9, Math.toRadians(90));           // 144-56=88
    private final Pose tagPose = new Pose(86, 39, Math.toRadians(90));            // Same Y, same heading
    private final Pose shootPose = new Pose(86, 17.2, Math.toRadians(68));          // 180-112=68 - Y=17
    private final Pose rotatedPose = new Pose(90, 47, Math.toRadians(0));       // 144-52=92, 180-180=0
    private final Pose rightPose = new Pose(140, 47, Math.toRadians(0));        // 144-20=124
    private final Pose intermediatePose = new Pose(110, 24, Math.toRadians(0)
    ); // Y closer to humanPlayerPose to avoid backwards motion
    private final Pose humanPlayerPose = new Pose(120, 24, Math.toRadians(0));  // 144-25=119
    private final Pose endPose = new Pose(90, 35, Math.toRadians(68));            // Same as shootPose heading
    private final double waitTime = 4.5;
    private final double pickupWaitTime = 1.7;

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
        telemetry.addData("Current Pos", follower.getPose().getX() + " " + follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void init_loop()
    {
        super.init_loop();
        Pose currentPose = follower.getPose();
        telemetry.addData("=== CURRENT POSITION ===", "");
        telemetry.addData("X Position", "%.2f", currentPose.getX());
        telemetry.addData("Y Position", "%.2f", currentPose.getY());
        telemetry.addData("Heading (degrees)", "%.2f", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Heading (radians)", "%.4f", currentPose.getHeading());
        telemetry.addData("", "");
        telemetry.addData("💡 Tip", "Move robot to find positions");
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

        // Path 4: Return to shoot position - control points for Y=17 shootPose
        returnToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        rightPose,           // (140, 45, 0°)
                        new Pose(115, 35),   // Control point 1
                        new Pose(95, 25),    // Control point 2
                        shootPose            // (86, 17, 68°)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();

        // Path 5: Go to intermediate position (heading now -15°)
        toIntermediate = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intermediatePose))
                .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();

        // Path 6: Go to human player position - VERY SLOW (heading -15°)
        PathConstraints verySlowConstraints = new PathConstraints(0.15, 20, 0.3, 0.3);
        Path toHumanPlayerPath = new Path(new BezierLine(intermediatePose, humanPlayerPose), verySlowConstraints);
        toHumanPlayerPath.setConstantHeadingInterpolation(Math.toRadians(-45));
        toHumanPlayer = follower.pathBuilder()
                .addPath(toHumanPlayerPath)
                .build();

        // Path 7: Return from human player to shoot (heading -15° to 68°)
        returnFromHuman = follower.pathBuilder()
                .addPath(new BezierCurve(
                        humanPlayerPose,     // (165, 12, -15°)
                        new Pose(125, 24),   // Control point 1 (adjusted for new Y)
                        new Pose(100, 20),   // Control point 2 (adjusted for new Y)
                        shootPose            // (86, 17, 68°)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(68))
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
