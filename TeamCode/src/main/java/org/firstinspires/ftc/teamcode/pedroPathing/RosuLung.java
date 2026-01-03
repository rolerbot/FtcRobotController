
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "RosuLung", group = "Pedro Pathing")
public class RosuLung extends OpMode {
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
    private int currentWaitPosition = 0; // 0 = not waiting, 1-3 = wait positions
    private boolean shootingStarted = false;
    private boolean motorsStarted = false;
    private static final double WAIT_TIME = 4.3;

    // Wait positions - will be set in buildPaths()
    private Pose waitPos1;
    private Pose waitPos2;
    private Pose waitPos3;

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

        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Position", "X=56, Y=8, H=90°");
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

        // Check for wait position (generic check)
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

        telemetry.addData("=== MIXER ===", "");
        telemetry.addData("Artifacts", mixer.GetArtifactCount());
        telemetry.addData("Wait Pos", currentWaitPosition);
        telemetry.addData("Shooting", shootingStarted);

        // Shooting logic at wait positions
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

    private Pose getTargetWaitPosition() {
        if (waitPos1 != null) return waitPos1;
        if (waitPos2 != null) return waitPos2;
        if (waitPos3 != null) return waitPos3;
        return null;
    }

    private int getNextWaitPosition() {
        if (currentWaitPosition == 0) return 1;
        if (currentWaitPosition == 1) return 2;
        if (currentWaitPosition == 2) return 3;
        return 0;
    }

    /**
     * Build paths with rotation compensation and proper constraints
     * - rotationConstraints (0.7 power) for paths WITH rotation
     * - straightConstraints (0.3 power) for paths WITHOUT rotation
     * - waitConstraints (0.15 power) for wait positions
     * - Compensation: ~6-9 inches for rotations >40°
     */
    private void buildPaths() {
        final double ROTATION_COMPENSATION = 6.0;

        PathConstraints rotationConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);
        PathConstraints straightConstraints = new PathConstraints(0.3, 30, 0.5, 0.5);
        PathConstraints waitConstraints = new PathConstraints(0.15, 15, 0.3, 0.3);

        // Set wait positions (mirrored for red side: 144-56=88, but using 87 for consistency)
        waitPos1 = new Pose(87, 15, Math.toRadians(68));
        waitPos2 = null;
        waitPos3 = null;

        pathChain = follower.pathBuilder()
            // Path 1: 90° → 68° (22° rotation, mirrored from 112°)
            .addPath(new BezierLine(
                new Pose(87, 9),
                new Pose(87, 15)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
            .setConstraints(straightConstraints)

            // Wait 1: Shooting position
            .addPath(new BezierLine(
                new Pose(87, 15, Math.toRadians(68)),
                new Pose(87, 15, Math.toRadians(68))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(68))
            .setConstraints(waitConstraints)

            // Path 2: 90° → 0° (90° rotation, mirrored from 180°, needs compensation)
            .addPath(new BezierCurve(
                new Pose(87, 15),
                new Pose(90, 23),
                new Pose(93, 30),
                new Pose(94, 36 - ROTATION_COMPENSATION)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
            .setConstraints(rotationConstraints)

            // Path 3: Move right (no rotation, mirrored from left)
            .addPath(new BezierLine(
                new Pose(94, 30, Math.toRadians(0)),
                new Pose(126, 30, Math.toRadians(0))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .setConstraints(straightConstraints)

            // Path 4: Return and rotate 0° → 68° (68° rotation, mirrored from 180° → 112°)
            .addPath(new BezierCurve(
                new Pose(126, 30),
                new Pose(113, 25),
                new Pose(100, 20),
                new Pose(87, 15)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
            .setConstraints(rotationConstraints)

            // Path 5: 68° → 0° (68° rotation, mirrored from 112° → 180°)
            .addPath(new BezierCurve(
                new Pose(87, 15),
                new Pose(95, 14),
                new Pose(103, 13),
                new Pose(108, 14 - ROTATION_COMPENSATION)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
            .setConstraints(rotationConstraints)

            // Path 6: Move right (no rotation, mirrored from left)
            .addPath(new BezierLine(
                new Pose(108, 8, Math.toRadians(0)),
                new Pose(139, 8, Math.toRadians(0))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .setConstraints(straightConstraints)

            // Path 7: Return with rotation 0° → 68° (68° rotation, mirrored from 180° → 112°)
            .addPath(new BezierCurve(
                new Pose(139, 8),
                new Pose(123, 10),
                new Pose(105, 12),
                new Pose(87, 15)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
            .setConstraints(rotationConstraints)

            // Path 8: Final position (no rotation, mirrored)
            .addPath(new BezierLine(
                new Pose(87, 15, Math.toRadians(68)),
                new Pose(103, 20, Math.toRadians(68))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(68))
            .setConstraints(rotationConstraints)

            .build();
    }
}

