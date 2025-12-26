package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
/*import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous(name = "Pedro Auto")
public class PedroOdometry implements Subsystem
{
    public void Run() {
    }
    public void LinkComponents(HardwareMap hwMap) {
    }
    public void Initialize(HardwareMap hwMap) {
    }
    /*private Follower follower;

    public void init()
    {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    public void start()
    {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(0, 0, Point.CARTESIAN),
                        new Point(20, 20, Point.CARTESIAN)
                ))
                .build();

        follower.followPath(path);
    }

    public void loop()
    {
        follower.update();
    }*/
}
