package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "Position Finder", group = "Pedro Pathing")
public class PositionFinder extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(86, 9, Math.toRadians(90));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "✓ Ready");
        telemetry.addData("Instructions", "Press START then move robot");
        telemetry.addData("Starting Pose", "X=86, Y=9, H=90°");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset to starting position when START is pressed
        follower.setStartingPose(startPose);
        telemetry.addData("Status", "Position reset to start");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower to get current odometry data
        follower.update();

        Pose currentPose = follower.getPose();

        telemetry.addData("╔════════════════════════════╗", "");
        telemetry.addData("║  CURRENT POSITION FINDER  ║", "");
        telemetry.addData("╚════════════════════════════╝", "");
        telemetry.addData("", "");
        telemetry.addData("X Position", "%.2f inches", currentPose.getX());
        telemetry.addData("Y Position", "%.2f inches", currentPose.getY());
        telemetry.addData("", "");
        telemetry.addData("Heading (degrees)", "%.2f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Heading (radians)", "%.4f", currentPose.getHeading());
        telemetry.addData("", "");
        telemetry.addData("━━━━━━━━━━━━━━━━━━━━━━━━━━━━", "");
        telemetry.addData("💡 Tip", "Move robot to desired position");
        telemetry.addData("📝 Note", "Write down the coordinates above");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Do nothing - no motors to stop
    }
}