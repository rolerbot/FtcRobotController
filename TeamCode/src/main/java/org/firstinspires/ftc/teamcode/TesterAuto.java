package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import  com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class TesterAuto extends OpMode
{
    Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState
    {
        Drive_StartPos_Shoot_Pos,
        Shoot_Preload;
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.333333333333332, 121.80645161290323, Math.toRadians(140));
    private final Pose shootPose = new Pose(49.909, 99.080, Math.toRadians(140));

    private PathChain driveStartPosShootPos;

    public void buildPaths()
    {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate()
    {
        switch (pathState)
        {
            case Drive_StartPos_Shoot_Pos:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.Shoot_Preload); // Resete the timer and make new state
                break;
            case Shoot_Preload:
                //Check is follower is done
                if (!follower.isBusy())
                    telemetry.addLine("Done state Path1");
                break;
            default:
                telemetry.addLine("Unknown path state");
                break;
        }
    }

    public void setPathState(PathState newState)
    {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init()
    {
        pathState = PathState.Drive_StartPos_Shoot_Pos;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO: Add in any other init machanisms

        buildPaths();
        follower.setPose(startPose);
    }

    public void start()
    {
        opModeTimer.resetTimer();
        setPathState(pathState);

    }

    @Override
    public void loop()
    {
        follower.update();
        statePathUpdate();

        telemetry.addData("OpMode Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}