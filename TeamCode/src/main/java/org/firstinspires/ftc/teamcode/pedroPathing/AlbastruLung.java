
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "AlbastruLung", group = "Pedro Pathing")
public class AlbastruLung extends OpMode {
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

        follower.setStartingPose(new Pose(56, 9, Math.toRadians(90)));

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

        // Set wait positions (these will be checked in loop)
        waitPos1 = new Pose(56, 15, Math.toRadians(112));
        waitPos2 = null; // No second wait in this path
        waitPos3 = null; // No third wait in this path

        pathChain = follower.pathBuilder()
            // Path 1: 90° → 112° (22° rotation)
            .addPath(new BezierLine(
                new Pose(56, 9),
                new Pose(56, 15)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
            .setConstraints(straightConstraints) // Small rotation

            // Wait 1: Shooting position
            .addPath(new BezierLine(
                new Pose(56, 15, Math.toRadians(112)),
                new Pose(56, 15, Math.toRadians(112))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(112))
            .setConstraints(waitConstraints)

            // Path 2: 90° → 180° (90° rotation, needs compensation)
            .addPath(new BezierCurve(
                new Pose(56, 15),
                new Pose(53, 23),
                new Pose(50, 30),
                new Pose(49, 36 - ROTATION_COMPENSATION) // Compensated from 36
            ))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .setConstraints(rotationConstraints)

            // Path 3: Move left (no rotation)
            .addPath(new BezierLine(
                new Pose(49, 30, Math.toRadians(180)),
                new Pose(17, 30, Math.toRadians(180))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setConstraints(straightConstraints)

            // Path 4: Return and rotate 180° → 112° (68° rotation)
            .addPath(new BezierCurve(
                new Pose(17, 30),
                new Pose(30, 25),
                new Pose(43, 20),
                new Pose(56, 15)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
            .setConstraints(rotationConstraints)

            // Path 5: 112° → 180° (68° rotation)
            .addPath(new BezierCurve(
                new Pose(56, 15),
                new Pose(48, 14),
                new Pose(40, 13),
                new Pose(35, 14 - ROTATION_COMPENSATION)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
            .setConstraints(rotationConstraints)

            // Path 6: Move left (no rotation)
            .addPath(new BezierLine(
                new Pose(35, 8, Math.toRadians(180)),
                new Pose(4, 8, Math.toRadians(180))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setConstraints(straightConstraints)

            // Path 7: Return with rotation 180° → 112° (68° rotation)
            .addPath(new BezierCurve(
                new Pose(4, 8),
                new Pose(20, 10),
                new Pose(38, 12),
                new Pose(56, 15)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
            .setConstraints(rotationConstraints)

            // Path 8: Final position (no rotation)
            .addPath(new BezierLine(
                new Pose(56, 15, Math.toRadians(112)),
                new Pose(40, 20, Math.toRadians(112))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(112))
            .setConstraints(rotationConstraints)

            .build();
    }
}

