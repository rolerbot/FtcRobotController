package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "DEBUG - Forward/Back Test", group = "Debug")
public class StrafeTest extends OpMode {
    private Follower follower;
    private int pathState;
    private Timer pathTimer;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose forwardPose = new Pose(0, 30, Math.toRadians(90));   // Forward 30"
    private final Pose backPose = new Pose(0, -30, Math.toRadians(90));     // Backward 30"
    private final Pose returnPose = new Pose(0, 0, Math.toRadians(90));

    private PathChain goForward, returnFromForward, goBackward, returnFromBackward;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("═══════════════════", "");
        telemetry.addData("TEST", "Forward/Backward");
        telemetry.addData("═══════════════════", "");
        telemetry.addData("1", "Go FORWARD 30\"");
        telemetry.addData("2", "Return (go BACK)");
        telemetry.addData("3", "Go BACKWARD 30\"");
        telemetry.addData("4", "Return (go FORWARD)");
        telemetry.addData("", "");
        telemetry.addData("⚠️ CRITICAL", "Mark start position!");
        telemetry.addData("", "Watch for drift!");
        telemetry.update();
    }

    public void buildPaths() {
        goForward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, forwardPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        returnFromForward = follower.pathBuilder()
                .addPath(new BezierLine(forwardPose, returnPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        goBackward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, backPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        returnFromBackward = follower.pathBuilder()
                .addPath(new BezierLine(backPose, returnPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", String.format("%.2f\"", follower.getPose().getX()));
        telemetry.addData("Y", String.format("%.2f\"", follower.getPose().getY()));
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                telemetry.addData("▶", "Going FORWARD 30\"");
                follower.followPath(goForward, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    telemetry.addData("✅", "At FORWARD position");
                    telemetry.addData("Expected", "Y=30");
                    telemetry.addData("Actual", String.format("Y=%.2f", follower.getPose().getY()));
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    telemetry.addData("▶", "Going BACK to start");
                    follower.followPath(returnFromForward, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    double errorY = Math.abs(follower.getPose().getY() - 0);

                    telemetry.addData("✅", "Returned from FORWARD");
                    telemetry.addData("Expected", "Y=0");
                    telemetry.addData("Actual", String.format("Y=%.2f", follower.getPose().getY()));
                    telemetry.addData("Error", String.format("%.2f\"", errorY));

                    if (errorY > 1.0) {
                        telemetry.addData("⚠️", "BACKWARD drift detected!");
                    }

                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 3.0) {
                    telemetry.addData("▶", "Going BACKWARD 30\"");
                    follower.followPath(goBackward, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    telemetry.addData("✅", "At BACKWARD position");
                    telemetry.addData("Expected", "Y=-30");
                    telemetry.addData("Actual", String.format("Y=%.2f", follower.getPose().getY()));
                    setPathState(6);
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    telemetry.addData("▶", "Going FORWARD to start");
                    follower.followPath(returnFromBackward, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    double errorY = Math.abs(follower.getPose().getY() - 0);

                    telemetry.addData("✅", "Returned from BACKWARD");
                    telemetry.addData("Expected", "Y=0");
                    telemetry.addData("Actual", String.format("Y=%.2f", follower.getPose().getY()));
                    telemetry.addData("Error", String.format("%.2f\"", errorY));

                    if (errorY > 1.0) {
                        telemetry.addData("⚠️", "FORWARD drift detected!");
                    }

                    telemetry.addData("═══════════════════", "");
                    telemetry.addData("🏁", "TEST COMPLETE");
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}