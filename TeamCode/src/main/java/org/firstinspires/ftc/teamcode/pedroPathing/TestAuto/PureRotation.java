package org.firstinspires.ftc.teamcode.pedroPathing.TestAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    @Autonomous(name = "Pure Rotation Test")
public class PureRotation extends LinearOpMode
    {
        @Override
        public void runOpMode()
        {
            Follower follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(125.992, 121.402, Math.toRadians(126)));

            waitForStart();

            // Rotate from 126° to 70° WITHOUT moving
            follower.followPath(follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(125.992, 121.402),
                            new Pose(125.992, 121.402) // SAME position
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(70))
                    .build());

            while (follower.isBusy() && opModeIsActive())
            {
                follower.update();
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.update();
            }

            Pose finalPose = follower.getPose();

            telemetry.addData("=== FINAL POSITION ===", "");
            telemetry.addData("Start X", 125.992);
            telemetry.addData("Final X", finalPose.getX());
            telemetry.addData("X Error", finalPose.getX() - 125.992);
            telemetry.addData("", "");
            telemetry.addData("Start Y", 121.402);
            telemetry.addData("Final Y", finalPose.getY());
            telemetry.addData("Y Error", finalPose.getY() - 121.402);
            telemetry.addData("", "");
            telemetry.addData("Final Heading", Math.toDegrees(finalPose.getHeading()));
            telemetry.update();

            sleep(30000);
        }
}
