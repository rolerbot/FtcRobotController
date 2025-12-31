package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Husky;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Mixer;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.TelemetryCustom;

@Autonomous(name = "Shooting Trajectory", group = "Pedro Pathing")
public class ShootingTrajectory extends OpMode {
    private Follower follower;
    private PathChain pathChain;
    private Mixer mixer;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private TelemetryCustom tl;
    private boolean pathsBuilt = false;
    private ElapsedTime waitTimer = new ElapsedTime();
    private boolean canArrange = false;
    private int currentWaitPosition = 0; // 0 = not waiting, 1 = first wait, 2 = second wait, 3 = third wait
    private boolean shootingStarted = false;
    private boolean motorsStarted = false; // Track if shooter motors have been started
    private static final double WAIT_TIME = 4.3; // 4 second wait time from trajectory

    // Wait positions from trajectory
    private static final Pose WAIT_POS_1 = new Pose(67.09, 8.60, Math.toRadians(109));
    private static final Pose WAIT_POS_2 = new Pose(71.23, 77.94, Math.toRadians(140));
    private static final Pose WAIT_POS_3 = new Pose(72.43, 82.41, Math.toRadians(140));

    double X = 67.09677419354838; double Y = 8.602150537634405;

    @Override
    public void init() {
        // Initialize TelemetryCustom first
        tl = new TelemetryCustom(telemetry);

        // Initialize components using autonomous constructors (no gamepads)
        intake = new Intake(tl);
        intake.Initialize(hardwareMap);

        mixer = new Mixer(tl, intake);
        mixer.Initialize(hardwareMap);

        husky = new Husky(tl);
        husky.Initialize(hardwareMap);

        // Initialize shooter for autonomous
        shooter = new Shooter(tl, mixer, intake, husky);
        shooter.Initialize(hardwareMap);

        // Initialize follower and set starting pose
        follower = Constants.createFollower(hardwareMap);

        // REDUCE OVERSHOOT: Lower max power for better control
        // Helps with tracking during rotation
        follower.setMaxPower(0.4); // Reduce to 45% for rotation accuracy

        intake.SetForward();
        mixer.ResetServoPosition();

        // Starting position from trajectory.pp: x=56, y=8, heading=90 degrees
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Position", "X=56, Y=8, H=90°");
        telemetry.addData("Max Power", "60% (Overshoot Prevention)");
        telemetry.addData("Shooting", "Enabled");
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
            telemetry.addData("Status", "Following trajectory");
        } else {
            telemetry.addData("Error", "Path is null!");
        }
        intake.SetPowerMax();
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        // Update subsystems - mixer runs ONLY when NOT shooting!
        if (currentWaitPosition == 0) {
            mixer.Run(); // Collect balls only when NOT at wait position

            // PREPARE MOTORS: Start spinning when first ball is detected
            if (!motorsStarted && mixer.GetArtifactCount() > 0) {
                shooter.SetShooterVelocity(shooter.GetMotorPower());
                motorsStarted = true;
                telemetry.addData("⚡ Motors", "STARTED - Preparing!");
            }
        }
        husky.Run();

        Pose currentPose = follower.getPose();

        // Calculate distance to wait position 1 (NEVER use == for doubles!)
        double distToWait = Math.hypot(currentPose.getX() - X, currentPose.getY() - Y);

        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("📍 X", "%.2f", currentPose.getX());
        telemetry.addData("📍 Y", "%.2f", currentPose.getY());
        telemetry.addData("📍 Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("🎯 Dist to Wait", "%.2f in", distToWait);

        telemetry.addData("=== MIXER STATUS ===", "");
        telemetry.addData("Artifacts", mixer.GetArtifactCount());
        telemetry.addData("Purple Count", mixer.GetCountPurple());
        telemetry.addData("Green Count", mixer.GetCountGreen());
        telemetry.addData("Mixer Empty", mixer.IsEmpty());

        telemetry.addData("=== SHOOTING ===", "");
        telemetry.addData("Wait Pos", currentWaitPosition);
        telemetry.addData("Shooting Started", shootingStarted);
        telemetry.addData("Motors Started", motorsStarted);
        telemetry.addData("Auto Shooting", shooter.IsAutoShooting());

        // Check if at wait position (within 10 inches tolerance)
        if (distToWait < 10.0) {
            if (!shootingStarted) {
                // Just arrived at wait position - start shooting
                currentWaitPosition = 1;
                waitTimer.reset();
                shootingStarted = true;
                shooter.StartAutoShoot(); // Start shooting sequence
                telemetry.addData("🔴 STATUS", "WAIT DETECTED!");
            }

            // CRITICAL: Call AutoShoot() EVERY loop while waiting!
            boolean shootingDone = shooter.AutoShoot();

            // Check if wait time complete OR shooting done
            if (waitTimer.seconds() >= WAIT_TIME || shootingDone) {
                currentWaitPosition = 0;
                shootingStarted = false;
                telemetry.addData("✅ STATUS", "Shooting Complete!");
            } else {
                telemetry.addData("⏳ STATUS", String.format("Wait %.2fs/%.2fs", waitTimer.seconds(), WAIT_TIME));
            }
        } else {
            if (currentWaitPosition == 0) {
                telemetry.addData("STATUS", "Following path...");
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        shooter.StopShooterMotors();
    }

    /**
     * Build paths from trajectory (1).pp:
     * Path 1: (56, 8) → (56, 36) heading 90°→180°
     * Path 2: (56, 36) → (19.27, 35.61) tangential
     * Path 3: (19.27, 35.61) → (67.09, 8.60) heading 180°→109°
     * Wait 4: WAIT 4s at (67.09, 8.60) - SHOOT HERE!
     * Path 5: (67.09, 8.60) → (43.01, 61.08) heading 109°→180°
     * Path 6: (43.01, 61.08) → (18.92, 60.39) tangential
     * Path 7: (18.92, 60.39) → (71.23, 77.94) heading 180°→140°
     * Wait 8: WAIT 4s at (71.23, 77.94) - SHOOT HERE!
     * Path 9: (71.23, 77.94) → (19.10, 83.78) heading 140°→180°
     * Path 10: (19.10, 83.78) → (72.43, 82.41) heading 180°→140°
     * Wait 11: WAIT 4s at (72.43, 82.41) - SHOOT HERE!
     */
    private void buildPaths() {
        // Slow constraints to prevent overshoot
        PathConstraints slowConstraints = new PathConstraints(0.25, 25, 0.4, 0.4);

        // ULTRA slow constraints for WAIT positions - CRITICAL!
        PathConstraints waitConstraints = new PathConstraints(0.15, 15, 0.3, 0.3);

        pathChain = follower.pathBuilder()
            // Path 1: Forward movement WITH rotation
            .addPath(new BezierLine(
                new Pose(56, 8, Math.toRadians(90)),
                new Pose(56, 27, Math.toRadians(180))
            ))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .setConstraints(slowConstraints)

            // Path 2: Move left (REDUCED to half distance)
            .addPath(new BezierLine(
                new Pose(56, 27, Math.toRadians(180)),
                new Pose(35, 27, Math.toRadians(180))  // End at X=43
            ))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setConstraints(slowConstraints)

            // Path 3: Move to first wait position (MUST start where Path 2 ends!)
            .addPath(new BezierLine(
                new Pose(33, 27, Math.toRadians(180)),  // ✓ FIXED: Start at X=43
                new Pose(56, 8, Math.toRadians(130))
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
            .setConstraints(slowConstraints)

            // Wait 4: Stay at position for shooting - CRITICAL FIX!
            .addPath(new BezierLine(
                new Pose(56, 8, Math.toRadians(130)),
                new Pose(56, 8, Math.toRadians(130))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(130))
            .setConstraints(waitConstraints) // ✓ ADĂUGAT!

            // Path 5: Move to upper area
            .addPath(new BezierLine(
                new Pose(56, 8, Math.toRadians(130)),
                new Pose(43.01075268817205, 61.075268817204304, Math.toRadians(180))
            ))
            .setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))
            .setConstraints(slowConstraints)

            // Path 6: Move left
            .addPath(new BezierLine(
                new Pose(43.01075268817205, 61.075268817204304, Math.toRadians(180)),
                new Pose(18.9247311827957, 60.387096774193544, Math.toRadians(180))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setConstraints(slowConstraints)

            // Path 7: Move to second wait position
            .addPath(new BezierLine(
                new Pose(18.9247311827957, 60.387096774193544, Math.toRadians(180)),
                new Pose(71.2258064516129, 77.93548387096774, Math.toRadians(140))
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
            .setConstraints(slowConstraints)

            // Wait 8: Stay at position for shooting - CRITICAL FIX!
            .addPath(new BezierLine(
                new Pose(71.2258064516129, 77.93548387096774, Math.toRadians(140)),
                new Pose(71.2258064516129, 77.93548387096774, Math.toRadians(140))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(140))
            .setConstraints(waitConstraints) // ✓ ADĂUGAT!

            // Path 9: Move back left
            .addPath(new BezierLine(
                new Pose(71.2258064516129, 77.93548387096774, Math.toRadians(140)),
                new Pose(19.096774193548388, 83.78494623655914, Math.toRadians(180))
            ))
            .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
            .setConstraints(slowConstraints)

            // Path 10: Move to third wait position
            .addPath(new BezierLine(
                new Pose(19.096774193548388, 83.78494623655914, Math.toRadians(180)),
                new Pose(72.43010752688173, 82.40860215053763, Math.toRadians(140))
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
            .setConstraints(slowConstraints)

            // Wait 11: Stay at position for final shooting - CRITICAL FIX!
            .addPath(new BezierLine(
                new Pose(72.43010752688173, 82.40860215053763, Math.toRadians(140)),
                new Pose(72.43010752688173, 82.40860215053763, Math.toRadians(140))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(140))
            .setConstraints(waitConstraints) // ✓ ADĂUGAT!

            .build();
    }
}

