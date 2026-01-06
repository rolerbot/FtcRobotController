
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "RosuScurtComplet", group = "Pedro Pathing")
public class RosuScurtComplet extends OpMode {
    private Follower follower;
    private PathChain pathChain;
    private Mixer mixer;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private RobotAllignment robotAllignment;
    private TelemetryCustom tl;
    private boolean pathsBuilt = false;
    private ElapsedTime waitTimer = new ElapsedTime();
    private int currentWaitPosition = 0;
    private boolean shootingStarted = false;
    private boolean motorsStarted = false;
    private static final double WAIT_TIME = 4.3;

    private Pose waitPos1, waitPos2, waitPos3;

    @Override
    public void init() {
        tl = new TelemetryCustom(telemetry);

        intake = new Intake(tl);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(tl, intake);
        mixer.Initialize(hardwareMap);

        husky = new Husky(tl);
        husky.Initialize(hardwareMap);

        robotAllignment = new RobotAllignment(tl, true);
        robotAllignment.Initialize(hardwareMap);

        shooter = new Shooter(tl, mixer, intake, husky, robotAllignment);
        shooter.Initialize(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.4);

        intake.SetForward();
        mixer.ResetServoPosition();

        follower.setStartingPose(new Pose(119, 129, Math.toRadians(143.5)));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Position", "X=25, Y=130.75, H=143.5°");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (!pathsBuilt) {
            buildPaths();
            pathsBuilt = true;
        }

        husky.Run();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Paths Built", pathsBuilt);
        telemetry.addData("Artifacts Detected", mixer.GetArtifactCount());
        telemetry.update();
    }

    @Override
    public void start() {
        if (pathChain != null) {
            follower.followPath(pathChain, true);
        }
        intake.SetPowerMax();
    }

    @Override
    public void loop() {
        follower.update();

        if (currentWaitPosition == 0) {
            mixer.Run();

            if (!motorsStarted && mixer.GetArtifactCount() > 0) {
                shooter.SetShooterVelocity(shooter.GetMotorPower());
                motorsStarted = true;
            }
        }
        husky.Run();

        Pose currentPose = follower.getPose();

        Pose targetWait = getTargetWaitPosition();
        double distToWait = 0;
        if (targetWait != null) {
            distToWait = Math.hypot(currentPose.getX() - targetWait.getX(), currentPose.getY() - targetWait.getY());
        }

        Vector velocity = follower.getVelocity();
        double speed = velocity.getMagnitude();

        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("📍 X", "%.2f", currentPose.getX());
        telemetry.addData("📍 Y", "%.2f", currentPose.getY());
        telemetry.addData("📍 Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("🚀 Speed", "%.2f in/s", speed);
        telemetry.addData("🎯 Dist to Wait", "%.2f in", distToWait);
        telemetry.addData("Path Following", follower.isBusy() ? "BUSY" : "IDLE");

        telemetry.addData("=== MIXER ===", "");
        telemetry.addData("Artifacts", mixer.GetArtifactCount());
        telemetry.addData("Wait Pos", currentWaitPosition);
        telemetry.addData("Shooting", shootingStarted);
        telemetry.addData("Motors Started", motorsStarted);

        if (targetWait != null && distToWait < 10.0) {
            if (!shootingStarted) {
                currentWaitPosition = getNextWaitPosition();
                waitTimer.reset();
                shootingStarted = true;
                shooter.StartAutoShoot();
            }

            boolean shootingDone = shooter.AutoShoot();

            if (waitTimer.seconds() >= WAIT_TIME || shootingDone) {
                currentWaitPosition = 0;
                shootingStarted = false;
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        shooter.StopShooterMotors();
    }

    private Pose getTargetWaitPosition()
    {
        if (currentWaitPosition == 0 && waitPos1 != null) return waitPos1;
        if (currentWaitPosition == 1 && waitPos2 != null) return waitPos2;
        if (currentWaitPosition == 2 && waitPos3 != null) return waitPos3;
        return null;
    }

    private int getNextWaitPosition()
    {
        if (currentWaitPosition == 0) return 1;
        if (currentWaitPosition == 1) return 2;
        if (currentWaitPosition == 2) return 3;
        return 0;
    }

    /**
     * Build paths with rotation compensation and proper constraints
     */
    private void buildPaths() {
        final double ROTATION_COMPENSATION = 6.0;

        PathConstraints rotationConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);
        PathConstraints straightConstraints = new PathConstraints(0.3, 30, 0.5, 0.5);
        PathConstraints waitConstraints = new PathConstraints(0.15, 15, 0.3, 0.3);

        // Wait positions (mirrored for red side)
        waitPos1 = new Pose(99.527, 100.172, Math.toRadians(36.5));
        waitPos2 = new Pose(99.527, 100.172, Math.toRadians(36.5));
        waitPos3 = new Pose(99.527, 100.172, Math.toRadians(36.5));

        pathChain = follower.pathBuilder()
            // Path 1: 36.5° → 120° (83.5° rotation - major rotation!, mirrored from 143.5° → 60°)
            .addPath(new BezierCurve(
                new Pose(119, 129),
                new Pose(111, 120),
                new Pose(105, 110),
                new Pose(99.527, 100.172 - ROTATION_COMPENSATION)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(120))
            .setConstraints(rotationConstraints)

            // Path 2: Rotate back 120° → 36.5° (stay in place)
            .addPath(new BezierLine(
                new Pose(99.527, 94, Math.toRadians(120)),
                new Pose(99.527, 94, Math.toRadians(36.5))
            ))
            .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(36.5))
            .setConstraints(waitConstraints)

            // Wait 1: First shooting position
            .addPath(new BezierLine(
                new Pose(99.527, 94, Math.toRadians(36.5)),
                new Pose(99.527, 94, Math.toRadians(36.5))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(36.5))
            .setConstraints(waitConstraints)

            // Path 3: 36.5° → 0° (36.5° rotation)
            .addPath(new BezierLine(
                new Pose(99.527, 94),
                new Pose(102.398, 84.473)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
            .setConstraints(straightConstraints)

            // Path 4: Move right (no rotation)
            .addPath(new BezierLine(
                new Pose(102.398, 84.473, Math.toRadians(0)),
                new Pose(126.710, 84.430, Math.toRadians(0))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .setConstraints(straightConstraints)

            // Path 5: Return 0° → 36.5° (36.5° rotation)
            .addPath(new BezierLine(
                new Pose(126.710, 84.430),
                new Pose(99.527, 94)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5))
            .setConstraints(straightConstraints)

            // Wait 2: Second shooting position
            .addPath(new BezierLine(
                new Pose(99.527, 94, Math.toRadians(36.5)),
                new Pose(99.527, 94, Math.toRadians(36.5))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(36.5))
            .setConstraints(waitConstraints)

            // Path 6: 36.5° → 0° (36.5° rotation)
            .addPath(new BezierLine(
                new Pose(99.527, 94),
                new Pose(100.419, 60.753)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
            .setConstraints(straightConstraints)

            // Path 7: Move right (no rotation)
            .addPath(new BezierLine(
                new Pose(100.419, 60.753, Math.toRadians(0)),
                new Pose(126.710, 60.054, Math.toRadians(0))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .setConstraints(straightConstraints)

            // Path 8: Return 0° → 36.5° (36.5° rotation)
            .addPath(new BezierLine(
                new Pose(126.710, 60.054),
                new Pose(99.527, 94)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36.5))
            .setConstraints(straightConstraints)

            // Wait 3: Third shooting position
            .addPath(new BezierLine(
                new Pose(99.527, 94, Math.toRadians(36.5)),
                new Pose(99.527, 94, Math.toRadians(36.5))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(36.5))
            .setConstraints(waitConstraints)

            // Path 9: 36.5° → 0° (36.5° rotation)
            .addPath(new BezierLine(
                new Pose(99.527, 94),
                new Pose(100.280, 35.656)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
            .setConstraints(straightConstraints)

            // Path 10: Move right (no rotation)
            .addPath(new BezierLine(
                new Pose(100.280, 35.656, Math.toRadians(0)),
                new Pose(127.624, 35.344, Math.toRadians(0))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .setConstraints(straightConstraints)

            // Path 11: Final path 0° → 68° (68° rotation, mirrored from 180° → 112°)
            .addPath(new BezierCurve(
                new Pose(127.624, 35.344),
                new Pose(113, 28),
                new Pose(100, 21),
                new Pose(87, 15)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
            .setConstraints(rotationConstraints)

            .build();
    }
}

