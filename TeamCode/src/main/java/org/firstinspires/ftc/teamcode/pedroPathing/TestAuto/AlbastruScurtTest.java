package org.firstinspires.ftc.teamcode.pedroPathing.TestAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name="AlbastruScurtTest", group="Robot")
public class AlbastruScurtTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double timeout = 0;

    //region PEDRO VARS
    private Follower follower;
    private Pose startPose, shoot1, movePoint;
    private Pose[] pickup1 = new Pose[3];
    private Pose[] pickup2 = new Pose[3];
    private Pose[] pickup3 = new Pose[3];
    private Pose[] gatePose = new Pose[3];
    private PathChain scorePath0, scorePath1, scorePath2, scorePath3, moveScore, gatePath, pickupPath1, pickupPath2, pickupPath3;
    //endregion

    private final PathConstraints shootConstraints = new PathConstraints(0.99, 100, 1.6, 1.5);
    private final PathConstraints gateConstraints = new PathConstraints(0.99, 100, 1.8, 1.7);

    public void createPoses(){
        startPose = new Pose(19.9,123.5,Math.toRadians(54));

        //0 is control point, 1 is endpoint
        pickup1[0] = new Pose(46.44,81.52,Math.toRadians(180));
        pickup1[1] = new Pose(17.5,84,Math.toRadians(180));

        gatePose[0] = new Pose(25.82,77.24,Math.toRadians(90));
        gatePose[1] = new Pose(14.62,75.3,Math.toRadians(90));

        pickup2[0] = new Pose(63.97,54.52,Math.toRadians(180));
        pickup2[1] = new Pose(10,58.36,Math.toRadians(180));
        //return from pickup
        pickup2[2] = new Pose(48.083, 54.73,Math.toRadians(180));

        pickup3[0] = new Pose(76.64,30.5,Math.toRadians(180));
        pickup3[1] = new Pose(10,35.58,Math.toRadians(180));

        shoot1 = new Pose(57.5,98.4,Math.toRadians(180));
        movePoint = new Pose(31,69.6,Math.toRadians(90));
    }

    public void createPaths(){
        scorePath0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shoot1))
                .setConstraints(shootConstraints)
                .setLinearHeadingInterpolation(startPose.getHeading(),shoot1.getHeading(), 0.65)
                .addParametricCallback(0.75, ()-> {
                    follower.setMaxPower(0.8);
                } )
                .build();

        pickupPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup1[0],pickup1[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.15,()->{
                    follower.setMaxPower(0.3);
                })
                .setTimeoutConstraint(500)
                .build();

        pickupPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup2[0],pickup2[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.38,()->{
                    follower.setMaxPower(0.3);
                })
                .setTimeoutConstraint(500)
                .build();

        pickupPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup3[0],pickup3[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.45,()->{
                    follower.setMaxPower(0.3);
                })
                .setTimeoutConstraint(500)
                .build();

        gatePath = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1[1],gatePose[0],gatePose[1]))
                .setConstraints(gateConstraints)
                .setLinearHeadingInterpolation(pickup1[1].getHeading(),gatePose[1].getHeading())
                .build();

        scorePath1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose[1],shoot1))
                .setConstraints(shootConstraints)
                .setLinearHeadingInterpolation(gatePose[1].getHeading(),shoot1.getHeading())
                .build();

        scorePath2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2[1],pickup2[2],shoot1))
                .setConstraints(shootConstraints)
                .setTranslationalConstraint(1.5)
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .build();

        scorePath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3[1],shoot1))
                .setConstraints(shootConstraints)
                .setTranslationalConstraint(1.5)
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.8, ()-> {
                            follower.setMaxPower(0.85);
                        }
                )
                .build();

        moveScore = follower.pathBuilder()
                .addPath(new BezierLine(shoot1,movePoint))
                .setLinearHeadingInterpolation(shoot1.getHeading(), movePoint.getHeading())
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int pathState = 0;
        boolean running = true;

        createPoses();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        createPaths();

        telemetry.addData("Status", "Initialized - Close Blue 12 Ball Test");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            follower.update();

            //region PATH STUFF
            if(!follower.isBusy() && runtime.milliseconds() > timeout){
                switch(pathState){
                    //region CYCLE ZERO
                    case 0:
                        follower.followPath(scorePath0,true);
                        timeout = runtime.milliseconds() + 500;
                        pathState++;
                        break;
                    //endregion

                    //region CYCLE ONE
                    case 1:
                        follower.followPath(pickupPath1,false);
                        timeout = 0;
                        pathState++;
                        break;

                    case 2:
                        follower.setMaxPower(1);
                        follower.followPath(gatePath,false);
                        timeout = 0;  // FIXED: removed the 1400ms timeout
                        pathState++;
                        break;

                    case 3:
                        follower.followPath(scorePath1,true);
                        timeout = 0;
                        pathState++;
                        break;
                    //endregion

                    //region CYCLE TWO
                    case 4:
                        follower.followPath(pickupPath2,false);
                        timeout = 0;
                        pathState++;
                        break;

                    case 5:
                        follower.setMaxPower(1);
                        follower.followPath(scorePath2,true);
                        timeout = 0;
                        pathState++;
                        break;
                    //endregion

                    //region CYCLE THREE
                    case 6:
                        follower.followPath(pickupPath3,false);
                        timeout = 0;
                        pathState++;
                        break;

                    case 7:
                        follower.setMaxPower(1);
                        follower.followPath(scorePath3,true);
                        timeout = 0;
                        pathState++;
                        break;
                    //endregion

                    case 8:
                        follower.followPath(moveScore);
                        timeout = 0;
                        pathState++;
                        running=false;
                        break;
                }
            }
            //endregion

            //region TELEMETRY
            if(!running) telemetry.addLine("Done!");

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
            //endregion
        }
    }
}