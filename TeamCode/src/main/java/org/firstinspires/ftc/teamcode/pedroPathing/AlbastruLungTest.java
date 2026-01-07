package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "AlbastruLungTest", group = "Pedro Pathing")
public class AlbastruLungTest extends OpMode {
    private Follower follower;
    private Path[] paths;
    private Mixer mixer;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private RobotAllignment robotAllignment;
    private TelemetryCustom tl;
    private boolean pathsBuilt = false;
    private ElapsedTime waitTimer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime transitionTimer = new ElapsedTime();
    private boolean shootingStarted = false;
    private static final double WAIT_TIME = 6.0;  // Wait up to 6 seconds for shooting
    private static final double TRANSITION_DELAY = 2.0;  // Wait 2 seconds after shooting before moving
    private enum AutoState {
        MOVING_TO_TAG,
        MOVING_TO_SHOOT,
        WAITING_SHOOT1,
        TRANSITION1,
        MOVING_LEFT1,
        RETURNING1,
        WAITING_SHOOT2,
        TRANSITION2,
        MOVING_LEFT2,
        RETURNING2,
        WAITING_SHOOT3,
        TRANSITION3,
        PARKING,
        DONE
    }
    private AutoState currentState = AutoState.MOVING_TO_TAG;
    @Override
    public void init() {
        tl = new TelemetryCustom(telemetry);
        intake = new Intake(tl);
        intake.Initialize(hardwareMap);
        mixer = new Mixer(tl, intake);
        mixer.Initialize(hardwareMap);
        husky = new Husky(tl);
        husky.Initialize(hardwareMap);
        robotAllignment = new RobotAllignment(tl, true, true);
        robotAllignment.Initialize(hardwareMap);
        shooter = new Shooter(tl, mixer, intake, husky, robotAllignment);
        shooter.Initialize(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.7);
        intake.SetForward();
        mixer.ResetServoPosition();
        mixer.SetArtifacts();
        Pose startPose = new Pose(56, 9, Math.toRadians(90));
        follower.setStartingPose(startPose);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // Ignore
        }
        follower.setPose(startPose);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Position", "X=56, Y=9, H=90?");
        telemetry.addData("Preloaded Samples", mixer.GetArtifactCount());
        telemetry.update();
    }
    @Override
    public void init_loop() {
        follower.update();
        if (!pathsBuilt) {
            buildPaths();
            pathsBuilt = true;
        }
        robotAllignment.Run();
        husky.Run();
        Pose currentPose = follower.getPose();
        telemetry.addData("Status", "Ready");
        telemetry.addData("Paths Built", pathsBuilt);
        telemetry.addData("", "");
        telemetry.addData("?? POSE CHECK", "");
        telemetry.addData("  X", "%.2f (should be ~56)", currentPose.getX());
        telemetry.addData("  Y", "%.2f (should be ~9)", currentPose.getY());
        telemetry.addData("  Heading", "%.1f? (should be ~90?)", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("", "");
        telemetry.addData("?? PRELOADED", "");
        telemetry.addData("  Artifacts", mixer.GetArtifactCount());
        double xError = Math.abs(currentPose.getX() - 56);
        double yError = Math.abs(currentPose.getY() - 9);
        if (xError > 5 || yError > 5) {
            telemetry.addData("??", "Resetting pose...");
            follower.setPose(new Pose(56, 9, Math.toRadians(90)));
        } else {
            telemetry.addData("?", "Pose OK!");
        }
        telemetry.update();
    }
    @Override
    public void start() {
        if (paths != null && paths.length > 0) {
            follower.followPath(paths[0], true);
            currentState = AutoState.MOVING_TO_TAG;
            stateTimer.reset();
        }
        intake.SetPowerMax();
    }
    @Override
    public void loop() {
        husky.Run();
        robotAllignment.Run();
        if (currentState != AutoState.WAITING_SHOOT1 &&
                currentState != AutoState.WAITING_SHOOT2 &&
                currentState != AutoState.WAITING_SHOOT3 &&
                currentState != AutoState.TRANSITION1 &&
                currentState != AutoState.TRANSITION2 &&
                currentState != AutoState.TRANSITION3) {
            follower.update();
        }
        if (currentState != AutoState.WAITING_SHOOT1 &&
                currentState != AutoState.WAITING_SHOOT2 &&
                currentState != AutoState.WAITING_SHOOT3 &&
                currentState != AutoState.TRANSITION1 &&
                currentState != AutoState.TRANSITION2 &&
                currentState != AutoState.TRANSITION3) {
            mixer.Run();
        }
        shooter.Run();
        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double speed = velocity.getMagnitude();
        telemetry.addData("????????????????????", "");
        telemetry.addData("?? STATE", currentState);
        telemetry.addData("?? Time in State", "%.2f sec", stateTimer.seconds());
        telemetry.addData("????????????????????", "");
        telemetry.addData("", "");
        telemetry.addData("?? POSITION", "");
        telemetry.addData("  X", "%.2f", currentPose.getX());
        telemetry.addData("  Y", "%.2f", currentPose.getY());
        telemetry.addData("  Heading", "%.1f?", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("  Speed", "%.2f in/s", speed);
        telemetry.addData("", "");
        telemetry.addData("?? FOLLOWER", "");
        telemetry.addData("  Is Busy?", follower.isBusy() ? "YES" : "NO");
        telemetry.addData("", "");
        telemetry.addData("?? SHOOTING", "");
        telemetry.addData("  Artifacts", mixer.GetArtifactCount());
        telemetry.addData("  Shooting?", shootingStarted);
        telemetry.addData("  Type", shooter.GetShootingType());
        telemetry.addData("  Allowed", shooter.GetShootingAllow());
        switch (currentState) {
            case MOVING_TO_TAG:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "Merge ?nainte la TAG (56, 39)");
                if (!follower.isBusy()) {
                    currentState = AutoState.MOVING_TO_SHOOT;
                    stateTimer.reset();
                    if (paths.length > 1) {
                        follower.followPath(paths[1], true);
                    }
                    telemetry.addData("?", "Tag v?zut! Merge ?napoi la shoot");
                }
                break;
            case MOVING_TO_SHOOT:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "Merge ?napoi la shoot (56, 15)");
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    currentState = AutoState.WAITING_SHOOT1;
                    stateTimer.reset();
                    waitTimer.reset();
                    shootingStarted = true;
                    shooter.TriggerAutonomousShooting();
                    telemetry.addData("?", "Arrived! SHOOTING!");
                }
                break;
            case WAITING_SHOOT1:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "ARUNC? TOATE MINGILE!");
                telemetry.addData("?? Timer", "%.2f / %.2f sec", waitTimer.seconds(), WAIT_TIME);
                telemetry.addData("  Mingi r?mase", mixer.GetArtifactCount());
                if (mixer.IsEmpty() || waitTimer.seconds() >= WAIT_TIME) {
                    shootingStarted = false;
                    follower.breakFollowing();
                    currentState = AutoState.TRANSITION1;
                    stateTimer.reset();
                    transitionTimer.reset();
                    telemetry.addData("?", "Aruncare complet?! A?teapt? 5s...");
                }
                break;
            case TRANSITION1:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "A?teapt? 5s dup? aruncare");
                telemetry.addData("?? Delay", "%.2f / %.2f sec", transitionTimer.seconds(), TRANSITION_DELAY);
                if (transitionTimer.seconds() >= TRANSITION_DELAY) {
                    currentState = AutoState.MOVING_LEFT1;
                    stateTimer.reset();
                    if (paths.length > 2) {
                        follower.followPath(paths[2], true);
                    }
                    telemetry.addData("?", "Merge ?n st?nga 1");
                }
                break;
            case MOVING_LEFT1:
                telemetry.addData("", "");
                telemetry.addData("➡️ ACTION", "Merge în stânga 1");

                if (!follower.isBusy()) {
                    // After rotation completes, start straight movement to collect balls
                    stateTimer.reset();
                    if (paths.length > 3) {
                        follower.followPath(paths[3], true);  // Straight movement path
                    }
                }

                // Check if straight movement is complete
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    currentState = AutoState.RETURNING1;
                    stateTimer.reset();
                    if (paths.length > 4) {
                        follower.followPath(paths[4], true);  // Return path
                    }
                    telemetry.addData("✅", "Se întoarce 1");
                }
                break;
            case RETURNING1:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "Se ?ntoarce 1");
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    currentState = AutoState.WAITING_SHOOT2;
                    stateTimer.reset();
                    waitTimer.reset();
                    shootingStarted = true;
                    shooter.TriggerAutonomousShooting();
                    telemetry.addData("?", "SHOOTING 2!");
                }
                break;
            case WAITING_SHOOT2:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "ARUNC? 2!");
                telemetry.addData("?? Timer", "%.2f / %.2f sec", waitTimer.seconds(), WAIT_TIME);
                telemetry.addData("  Mingi r?mase", mixer.GetArtifactCount());
                if (mixer.IsEmpty() || waitTimer.seconds() >= WAIT_TIME) {
                    shootingStarted = false;
                    follower.breakFollowing();
                    currentState = AutoState.TRANSITION2;
                    stateTimer.reset();
                    transitionTimer.reset();
                    telemetry.addData("?", "Aruncare 2 complet?! A?teapt? 5s...");
                }
                break;
            case TRANSITION2:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "A?teapt? 5s dup? aruncare 2");
                telemetry.addData("?? Delay", "%.2f / %.2f sec", transitionTimer.seconds(), TRANSITION_DELAY);
                if (transitionTimer.seconds() >= TRANSITION_DELAY) {
                    currentState = AutoState.MOVING_LEFT2;
                    stateTimer.reset();
                    if (paths.length > 5) {
                        follower.followPath(paths[5], true);  // Path 5: rotation
                    }
                    telemetry.addData("✅", "Merge în stânga 2");
                }
                break;
            case MOVING_LEFT2:
                telemetry.addData("", "");
                telemetry.addData("➡️ ACTION", "Merge în stânga 2");

                if (!follower.isBusy()) {
                    // After rotation completes, start straight movement
                    stateTimer.reset();
                    if (paths.length > 6) {
                        follower.followPath(paths[6], true);  // Straight movement path
                    }
                }

                // Check if straight movement is complete
                if (!follower.isBusy() && stateTimer.seconds() > 0.5) {
                    currentState = AutoState.RETURNING2;
                    stateTimer.reset();
                    if (paths.length > 7) {
                        follower.followPath(paths[7], true);  // Return path
                    }
                    telemetry.addData("✅", "Se întoarce 2");
                }
                break;
            case RETURNING2:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "Se ?ntoarce 2");
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    currentState = AutoState.WAITING_SHOOT3;
                    stateTimer.reset();
                    waitTimer.reset();
                    shootingStarted = true;
                    shooter.TriggerAutonomousShooting();
                    telemetry.addData("?", "SHOOTING 3!");
                }
                break;
            case WAITING_SHOOT3:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "ARUNC? 3!");
                telemetry.addData("?? Timer", "%.2f / %.2f sec", waitTimer.seconds(), WAIT_TIME);
                telemetry.addData("  Mingi r?mase", mixer.GetArtifactCount());
                if (mixer.IsEmpty() || waitTimer.seconds() >= WAIT_TIME) {
                    shootingStarted = false;
                    follower.breakFollowing();
                    currentState = AutoState.TRANSITION3;
                    stateTimer.reset();
                    transitionTimer.reset();
                    telemetry.addData("?", "Aruncare 3 complet?! A?teapt? 0.5s...");
                }
                break;
            case TRANSITION3:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "A?teapt? 5s dup? aruncare 3");
                telemetry.addData("?? Delay", "%.2f / %.2f sec", transitionTimer.seconds(), TRANSITION_DELAY);
                if (transitionTimer.seconds() >= TRANSITION_DELAY) {
                    currentState = AutoState.PARKING;
                    stateTimer.reset();
                    if (paths.length > 8) {
                        follower.followPath(paths[8], true);  // Path 8: parking
                    }
                    telemetry.addData("✅", "PARKING");
                }
                break;
            case PARKING:
                telemetry.addData("", "");
                telemetry.addData("?? ACTION", "Parking");
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                    stateTimer.reset();
                    telemetry.addData("?", "DONE!");
                }
                break;
            case DONE:
                telemetry.addData("", "");
                telemetry.addData("??? COMPLETE ???", "");
                telemetry.addData("Total Time", "%.2f seconds", stateTimer.seconds());
                break;
        }
        telemetry.update();
    }
    @Override
    public void stop() {
        follower.breakFollowing();
        shooter.StopShooterMotors();
    }
    private void buildPaths() {
        final double ROTATION_COMPENSATION = 6.0;

        PathConstraints rotationConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);
        PathConstraints straightConstraints = new PathConstraints(0.3, 30, 0.5, 0.5);

        paths = new Path[9];

        // Path 0: Move forward to TAG position (90° constant heading)
        paths[0] = new Path(new BezierLine(
                new Pose(56, 9, Math.toRadians(90)),
                new Pose(56, 39, Math.toRadians(90))
        ), straightConstraints);
        paths[0].setConstantHeadingInterpolation(Math.toRadians(90));

        // Path 1: Move back to shoot position (90° → 112°)
        paths[1] = new Path(new BezierLine(
                new Pose(56, 39, Math.toRadians(90)),
                new Pose(56, 15, Math.toRadians(112))
        ), straightConstraints);
        paths[1].setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112));

        // Path 2: 90° → 180° (90° rotation, needs compensation) - MATCHES LungTest
        paths[2] = new Path(new BezierCurve(
                new Pose(56, 15),
                new Pose(53, 23),
                new Pose(49, 30 - ROTATION_COMPENSATION)
        ), rotationConstraints);
        paths[2].setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180));

        // Path 3: Move left (180° constant, no rotation)
        paths[3] = new Path(new BezierLine(
                new Pose(49, 30, Math.toRadians(180)),
                new Pose(17, 30, Math.toRadians(180))
        ), straightConstraints);
        paths[3].setConstantHeadingInterpolation(Math.toRadians(180));

        // Path 4: Return and rotate 180° → 112° (68° rotation)
        paths[4] = new Path(new BezierCurve(
                new Pose(17, 30),
                new Pose(30, 25),
                new Pose(43, 20),
                new Pose(56, 15)
        ), rotationConstraints);
        paths[4].setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112));

        // Path 5: 112° → 180° (68° rotation, needs compensation)
        paths[5] = new Path(new BezierCurve(
                new Pose(56, 15),
                new Pose(48, 14),
                new Pose(40, 13),
                new Pose(35, 8 - ROTATION_COMPENSATION)
        ), rotationConstraints);
        paths[5].setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180));

        // Path 6: Move left (180° constant, no rotation)
        paths[6] = new Path(new BezierLine(
                new Pose(35, 8, Math.toRadians(180)),
                new Pose(4, 8, Math.toRadians(180))
        ), straightConstraints);
        paths[6].setConstantHeadingInterpolation(Math.toRadians(180));

        // Path 7: Return with rotation 180° → 112° (68° rotation)
        paths[7] = new Path(new BezierCurve(
                new Pose(4, 8),
                new Pose(20, 10),
                new Pose(38, 12),
                new Pose(56, 15)
        ), rotationConstraints);
        paths[7].setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112));

        // Path 8: Park (112° constant)
        paths[8] = new Path(new BezierLine(
                new Pose(56, 15, Math.toRadians(112)),
                new Pose(40, 20, Math.toRadians(112))
        ), rotationConstraints);
        paths[8].setConstantHeadingInterpolation(Math.toRadians(112));

        telemetry.addData("✅ All Paths Built", "9 paths ready");
        telemetry.update();
    }
}

