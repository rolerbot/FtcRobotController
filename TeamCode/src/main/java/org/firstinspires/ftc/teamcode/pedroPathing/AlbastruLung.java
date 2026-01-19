package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "AlbastruLung", group = "Pedro Pathing")
public class AlbastruLung extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private Mixer mixer;


    // Poses
    private final Pose startPose = new Pose(56, 9, Math.toRadians(90));
    private final Pose tagPose = new Pose(56, 39, Math.toRadians(90));
    private final Pose shootPose = new Pose(56, 15, Math.toRadians(112));
    private final Pose rotatedPose = new Pose(52, 28.5, Math.toRadians(180));
    private final Pose leftPose = new Pose(20, 28.5, Math.toRadians(180));      // Not as far left (was 17)
    private final Pose intermediatePose = new Pose(35, 9, Math.toRadians(-164.5));
    private final Pose humanPlayerPose = new Pose(25, 7, Math.toRadians(-164.5));
    private final Pose endPose = new Pose(56, 24, Math.toRadians(112));
    private final double waitTime = 4.5; // seconds - for shooting
    private final double pickupWaitTime = 2.25; // seconds - half time for ball pickup

    // Paths
    private Path toTag;
    private PathChain toShoot1, rotateLeft, goLeft, returnToShoot, toIntermediate, toHumanPlayer, returnFromHuman, toEnd;

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
        mixer.SetArtifacts(); // Load 3 balls

        // Pass null for robotAllignment - no IMU conflicts!
        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null);
        shooter.Initialize(hardwareMap);

        // NOW create follower and set starting pose AFTER subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Position", "X=56, Y=9, H=90°");
        telemetry.addData("Mixer", "3 balls loaded");
        telemetry.addData("Battery Voltage", String.format("%.2fV", shooter.GetBatteryVoltage()));
        telemetry.addData("Shooter F", "Compensated for battery");
        telemetry.update();
    }

    public void buildPaths() {
        // Define constraints
        PathConstraints normalConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);
        PathConstraints slowConstraints = new PathConstraints(0.3, 30, 0.4, 0.4);  // Slower to prevent slamming

        // Path 0: Forward to TAG (straight line, constant heading 90°)
        toTag = new Path(new BezierLine(startPose, tagPose));
        toTag.setConstantHeadingInterpolation(Math.toRadians(90));

        // Path 1: Back to shooting position (straight line, rotate 90° → 113°)
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(tagPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                .build();

        // Path 2: Small rotation move (113° → 180°)
        rotateLeft = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rotatedPose))
                .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                .build();

        // Path 3: Straight left movement (180° constant) - SLOW to prevent slamming
        Path goLeftPath = new Path(new BezierLine(rotatedPose, leftPose), slowConstraints);
        goLeftPath.setConstantHeadingInterpolation(Math.toRadians(180));
        goLeft = follower.pathBuilder()
                .addPath(goLeftPath)
                .build();

        // Path 4: Return to shoot position (curve, rotate 180° → 113°)
        returnToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        leftPose,
                        new Pose(30, 18),  // Adjusted control point Y from 22 to 18
                        new Pose(45, 16),  // Adjusted control point Y from 18 to 16
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                .build();

        // Path 5: Go to intermediate position (fast, normal constraints)
        toIntermediate = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intermediatePose))
                .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(-164.5))
                .build();

        // Path 6: Go to human player position (VERY SLOW)
        PathConstraints verySlowConstraints = new PathConstraints(0.15, 20, 0.3, 0.3);
        Path toHumanPlayerPath = new Path(new BezierLine(intermediatePose, humanPlayerPose), verySlowConstraints);
        toHumanPlayerPath.setConstantHeadingInterpolation(Math.toRadians(-164.5));
        toHumanPlayer = follower.pathBuilder()
                .addPath(toHumanPlayerPath)
                .build();

        // Path 7: Return from human player to shoot position
        returnFromHuman = follower.pathBuilder()
                .addPath(new BezierCurve(
                        humanPlayerPose,
                        new Pose(35, 13),  // Control point
                        new Pose(48, 14),  // Control point
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-164.5), Math.toRadians(112))
                .build();

        // Path 8: Go to end position for parking
        toEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setConstantHeadingInterpolation(Math.toRadians(112))
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
                    follower.setMaxPower(0.5); // Slower when going back from tag
                    follower.followPath(toShoot1, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for return to shoot position and check shooter
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8); // Reset to normal power
                    telemetry.addData("✅", "At shoot position - shooter spinning");
                    // Shooter.Run() already called in loop, PrepareLaunch sets velocity
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up, then start shooting
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                telemetry.addData("Shooter", "%.0f / %.0f RPM", currentRPM, targetRPM);

                // When RPM is close to target OR timeout, start shooting
                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.getElapsedTimeSeconds() > 2.0) {
                    telemetry.addData("✅", "Shooting!");
                    shooter.StartAutoShoot(); // Start autonomous shooting
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for shooting to complete
                if (shooter.AutoShoot()) { // Returns true when done
                    telemetry.addData("✅", "Done shooting!");
                    follower.followPath(rotateLeft, true);
                    setPathState(5);
                }
                break;

            case 5:
                // Wait for rotation, then start intake
                if (!follower.isBusy()) {
                    intake.SetPowerMax(); // Start intake
                    follower.setMaxPower(0.25);
                    follower.followPath(goLeft, true);
                    setPathState(6);
                }
                break;

            case 6:
                follower.setMaxPower(0.25);
                // Intake running, mixer collecting balls

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(returnToShoot, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Wait for return to shoot position
                if (!follower.isBusy()) {
                    telemetry.addData("✅", "Back at shoot - starting auto shoot");
                    shooter.StartAutoShoot(); // Start second shooting
                    setPathState(8);
                }
                break;

            case 8:
                // Wait for second shooting to complete
                if (shooter.AutoShoot()) {
                    telemetry.addData("✅", "Done second shooting!");
                    follower.followPath(toIntermediate, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Go to intermediate
                if (!follower.isBusy()) {
                    intake.SetPowerMax(); // Start intake for human player
                    follower.setMaxPower(0.2);
                    follower.followPath(toHumanPlayer, true);
                    setPathState(10);
                }
                break;

            case 10:
                follower.setMaxPower(0.2);
                // Intake running, collecting from human player

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    setPathState(11);
                }
                break;

            case 11:
                // Wait at human player - reduced wait time for ball pickup
                if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                    follower.followPath(returnFromHuman, true);
                    setPathState(12);
                }
                break;

            case 12:
                // Return to shoot
                if (!follower.isBusy()) {
                    telemetry.addData("✅", "Back - final shooting");
                    shooter.StartAutoShoot(); // Final shooting
                    setPathState(13);
                }
                break;

            case 13:
                // Wait for final shooting
                if (shooter.AutoShoot()) {
                    telemetry.addData("✅", "Done final shooting!");
                    intake.SetMotorPower(0.0); // Stop intake
                    follower.followPath(toEnd, true);
                    setPathState(14);
                }
                break;

            case 14:
                // Park
                if (!follower.isBusy()) {
                    telemetry.addData("✅✅✅", "COMPLETE!");
                    intake.SetMotorPower(0.0); // Ensure intake off
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
        intake.SetMotorPower(0.0); // Stop intake
    }
}

