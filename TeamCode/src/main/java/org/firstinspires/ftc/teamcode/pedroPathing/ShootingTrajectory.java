package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.*;
//hai sa facem sex toti

@Autonomous(name = "Shooting Trajectory", group = "Pedro Pathing")
public class ShootingTrajectory extends OpMode {
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
    private boolean canArrange = false;
    private int currentWaitPosition = 0; // 0 = not waiting, 1 = first wait, 2 = second wait, 3 = third wait
    private boolean shootingStarted = false;
    private boolean motorsStarted = false; // Track if shooter motors have been started
    private static final double WAIT_TIME = 4.3; // 4 second wait time from trajectory

    // Wait positions from trajectory (ORIGINAL - not used, kept for reference)
    private static final Pose WAIT_POS_1 = new Pose(67.09, 8.60, Math.toRadians(109));
    private static final Pose WAIT_POS_2 = new Pose(71.23, 77.94, Math.toRadians(140));
    private static final Pose WAIT_POS_3 = new Pose(72.43, 82.41, Math.toRadians(140));

    // Wait positions (COMPENSATED - matches actual path endpoints)
    private Pose waitPos1 = new Pose(50, 8, Math.toRadians(130));    // First wait (compensated from Path 3)
    private Pose waitPos2 = new Pose(66, 73, Math.toRadians(140));   // Second wait (compensated from Path 7)
    private Pose waitPos3 = new Pose(67, 80, Math.toRadians(140));   // Third wait (compensated from Path 10)

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

        //TelemetryCustom telemetry, GamepadEx ct1, GamepadEx ct2, Drivetrain drivetrain, boolean resetIMU
        robotAllignment = new RobotAllignment(tl, true, true);
        robotAllignment.Initialize(hardwareMap);

        // Initialize shooter for autonomous
        shooter = new Shooter(tl, mixer, intake, husky, robotAllignment);
        shooter.Initialize(hardwareMap);

        // Initialize follower and set starting pose
        follower = Constants.createFollower(hardwareMap);

        // REDUCE OVERSHOOT: Lower max power for better control
        // Helps with tracking during rotation
        follower.setMaxPower(0.4); // Reduce to 45% for rotation accuracy

        intake.SetForward();
        mixer.ResetServoPosition();

        // Starting position from trajectory.pp: x=56, y=8, heading=90 degrees
        follower.setStartingPose(new Pose(56, 9, Math.toRadians(90)));
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

        Pose currentPose = follower.getPose(); // ✓ REAL position from Pinpoint odometry!

        // Determine which wait position to check based on current wait state
        Pose targetWait = getTargetWaitPosition();
        double distToWait = 0;
        if (targetWait != null) {
            distToWait = Math.hypot(currentPose.getX() - targetWait.getX(), currentPose.getY() - targetWait.getY());
        }

        // CRITICAL: Get velocity to verify odometry is working
        Vector velocity = follower.getVelocity();
        double speed = velocity.getMagnitude();

        // Check if at wait position (within 10 inches tolerance)
        if (targetWait != null && distToWait < 10.0) {
            if (!shootingStarted) {
                // Just arrived at wait position - start shooting
                currentWaitPosition = getNextWaitPosition();
                waitTimer.reset();
                shootingStarted = true;
                shooter.StartAutoShoot(); // Start shooting sequence
                telemetry.addData("🔴 STATUS", String.format("WAIT %d DETECTED!", currentWaitPosition));
            }

            // CRITICAL: Call AutoShoot() EVERY loop while waiting!
            boolean shootingDone = shooter.AutoShoot();

            // Check if wait time complete OR shooting done
            if (waitTimer.seconds() >= WAIT_TIME || shootingDone) {
                currentWaitPosition = 0;
                shootingStarted = false;
                telemetry.addData("✅ STATUS", "Shooting Complete! Continuing trajectory...");
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
     * Get the current target wait position based on state
     * Returns null if not waiting (allows trajectory to continue)
     */
    private Pose getTargetWaitPosition() {
        if (currentWaitPosition == 1 && waitPos1 != null) return waitPos1;
        if (currentWaitPosition == 2 && waitPos2 != null) return waitPos2;
        if (currentWaitPosition == 3 && waitPos3 != null) return waitPos3;

        // Check if approaching any wait position (when currentWaitPosition == 0)
        if (currentWaitPosition == 0) {
            Pose currentPose = follower.getPose();

            // Check proximity to each wait position in order
            double dist1 = Math.hypot(currentPose.getX() - waitPos1.getX(), currentPose.getY() - waitPos1.getY());
            if (dist1 < 15.0) return waitPos1; // Approaching wait 1

            double dist2 = Math.hypot(currentPose.getX() - waitPos2.getX(), currentPose.getY() - waitPos2.getY());
            if (dist2 < 15.0) return waitPos2; // Approaching wait 2

            double dist3 = Math.hypot(currentPose.getX() - waitPos3.getX(), currentPose.getY() - waitPos3.getY());
            if (dist3 < 15.0) return waitPos3; // Approaching wait 3
        }

        return null; // Not at any wait position - trajectory continues
    }

    /**
     * Get next wait position number (1, 2, or 3)
     */
    private int getNextWaitPosition() {
        if (currentWaitPosition == 0) return 1;
        if (currentWaitPosition == 1) return 2;
        if (currentWaitPosition == 2) return 3;
        return 0;
    }

    /**
     * Build paths from trajectory (1).pp:
     *
     * ========== ROTATION COMPENSATION EXPLAINED ==========
     * When robot rotates while moving, the CENTER tracks correctly but the EDGES overshoot.
     * For an 18" x 18" robot rotating 90°:
     * - Center offset ≈ 9" * sin(90°) ≈ 9 inches
     * - BezierCurve makes rotation SMOOTHER but doesn't compensate for robot size
     * - Solution: Reduce target distance by ~6-8 inches when rotating >45°
     *
     * ========== CRITICAL: PINPOINT ODOMETRY PREVENTS DESYNC! ==========
     * Q: "Dacă reduc Y de la 36→27, dar robotul ajunge la 31, nu se pierde localizarea?"
     * A: NU! Pinpoint odometry măsoară poziția REALĂ independent de path:
     *
     * Exemplu:
     * 1. Path spune: "du-te la (56, 27)"
     * 2. Robot merge și ajunge EFECTIV la (56, 31) din cauza rotației
     * 3. Pinpoint DETECTEAZĂ (56, 31) - NU asumă (56, 27)!
     * 4. follower.update() actualizează follower.getPose() = (56, 31)
     * 5. Path următor PLEACĂ de la (56, 31), NU de la (56, 27)!
     *
     * Path-urile sunt doar SUGESTII (feedforward).
     * Pinpoint odometry este ADEVĂRUL (feedback cu closed-loop control).
     *
     * Verifică pe telemetrie că "📍 X/Y (REAL)" se actualizează continuu!
     * =========================================================
     *
     * TRANSFORMĂRI FĂCUTE (BezierLine → BezierCurve pentru rotații >40°):
     * ✅ Path 1: BezierCurve (90°→180°, 90° rotație) + compensare Y: 36→27
     * ✅ Path 2: BezierLine (0° rotație, nu e nevoie de curve)
     * ✅ Path 3: BezierCurve (180°→130°, 50° rotație)
     * ✅ Wait 4: BezierLine (stay in place)
     * ✅ Path 5: BezierCurve (130°→180°, 50° rotație)
     * ✅ Path 6: BezierLine (0° rotație)
     * ✅ Path 7: BezierCurve (180°→140°, 40° rotație)
     * ✅ Wait 8: BezierLine (stay in place)
     * ✅ Path 9: BezierCurve (140°→180°, 40° rotație) ← ADĂUGAT!
     * ✅ Path 10: BezierCurve (180°→140°, 40° rotație)
     * ✅ Wait 11: BezierLine (stay in place)
     *
     * TOTAL: 6 BezierCurve transformări pentru toate rotațiile ≥40°
     */
    private void buildPaths() {
        // ROTATION COMPENSATION: Robot is 18" x 18", half-length = 9"
        // When rotating 90°, compensate by ~6-8 inches on target position
        final double ROTATION_COMPENSATION = 6.0; // inches to reduce from target

        // Constraints for paths WITH rotation (higher power)
        PathConstraints rotationConstraints = new PathConstraints(0.7, 50, 0.7, 0.7);

        // Constraints for paths WITHOUT rotation (lower power for precision)
        PathConstraints straightConstraints = new PathConstraints(0.3, 30, 0.5, 0.5);

        // ULTRA slow constraints for WAIT positions - CRITICAL!
        PathConstraints waitConstraints = new PathConstraints(0.15, 15, 0.3, 0.3);

        pathChain = follower.pathBuilder()
            // Path 1: Forward movement WITH 90° rotation (COMPENSATED!)
            // Original target: Y=36, but robot overshoots due to rotation
            // Compensated: Y=27 (reduced by ~9 inches)
            // Using BezierCurve for SMOOTHER rotation (less overshoot than BezierLine)
            .addPath(new BezierCurve(
                new Pose(56, 8),   // Start
                new Pose(56, 15),  // Control 1: gradual forward
                new Pose(56, 22),  // Control 2: slow down before end
                new Pose(56, 27)   // End: compensated position
            ))
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .setConstraints(rotationConstraints)  // 0.7 power - has rotation

            // Path 2: Move left (NO rotation - 0°)
            .addPath(new BezierLine(
                new Pose(56, 27, Math.toRadians(180)),
                new Pose(35, 27, Math.toRadians(180))  // End at X=43
            ))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setConstraints(straightConstraints)  // 0.3 power - no rotation

            // Path 3: Smooth curve to first wait position (180° → 130°, 50° rotation)
            // COMPENSATED: X reduced from 56 to 50 (~6 inch for 50° rotation)
            .addPath(new BezierCurve(
                new Pose(33, 27),  // Start
                new Pose(40, 20),  // Control 1: smooth curve right
                new Pose(46, 12),  // Control 2: approach wait position (adjusted)
                new Pose(50, 8)    // End: COMPENSATED position (was 56)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .setConstraints(rotationConstraints)  // 0.7 power - has 50° rotation

            // Wait 4: Stay at position for shooting - CRITICAL FIX!
            .addPath(new BezierLine(
                new Pose(50, 8, Math.toRadians(130)),  // ✓ UPDATED to match Path 3
                new Pose(50, 8, Math.toRadians(130))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(130))
            .setConstraints(waitConstraints) // ✓ ADĂUGAT!

            // Path 5: Smooth curve to upper area (130° → 180°, 50° rotation)
            // COMPENSATED: Y reduced from 61 to 55 (~6 inch for 50° rotation)
            .addPath(new BezierCurve(
                new Pose(50, 8),    // Start: wait position (UPDATED from 56)
                new Pose(48, 25),   // Control 1: smooth exit from wait
                new Pose(45, 42),   // Control 2: arc towards target (adjusted)
                new Pose(43, 55)    // End: COMPENSATED position (was 61)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
            .setConstraints(rotationConstraints)  // 0.7 power - has 50° rotation

            // Path 6: Move left (NO rotation - 0°, no compensation needed)
            .addPath(new BezierLine(
                new Pose(43, 55, Math.toRadians(180)),  // ✓ UPDATED from Y=61
                new Pose(19, 55, Math.toRadians(180))   // ✓ Keep same Y
            ))
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setConstraints(straightConstraints)  // 0.3 power - no rotation

            // Path 7: Smooth curve to second wait position (180° → 140°, 40° rotation)
            // COMPENSATED: X reduced from 71 to 66 (~5 inch for 40° rotation)
            .addPath(new BezierCurve(
                new Pose(19, 55),    // Start: left side (UPDATED from Y=60)
                new Pose(35, 60),    // Control 1: smooth arc right
                new Pose(52, 68),    // Control 2: approach wait position (adjusted)
                new Pose(66, 73)     // End: COMPENSATED position (was X=71)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
            .setConstraints(rotationConstraints)  // 0.7 power - has 40° rotation

            // Wait 8: Stay at position for shooting - CRITICAL FIX!
            .addPath(new BezierLine(
                new Pose(66, 73, Math.toRadians(140)),  // ✓ UPDATED to match Path 7
                new Pose(66, 73, Math.toRadians(140))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(140))
            .setConstraints(waitConstraints) // ✓ ADĂUGAT!

            // Path 9: Smooth curve back left (140° → 180°, 40° rotation)
            // COMPENSATED: X increased from 19 to 25 (~6 inch for reverse direction)
            .addPath(new BezierCurve(
                new Pose(66, 73),    // Start: second wait position (UPDATED)
                new Pose(50, 76),    // Control 1: smooth arc left
                new Pose(35, 79),    // Control 2: approach target (adjusted)
                new Pose(25, 80)     // End: COMPENSATED position (was X=19)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
            .setConstraints(rotationConstraints)  // 0.7 power - has 40° rotation

            // Path 10: Smooth curve to third wait position (180° → 140°, 40° rotation)
            // COMPENSATED: X reduced from 72 to 67 (~5 inch for 40° rotation)
            .addPath(new BezierCurve(
                new Pose(25, 80),    // Start: left side (UPDATED from 19)
                new Pose(38, 81),    // Control 1: smooth arc right (adjusted)
                new Pose(52, 81),    // Control 2: approach wait position (adjusted)
                new Pose(67, 80)     // End: COMPENSATED position (was X=72)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
            .setConstraints(rotationConstraints)  // 0.7 power - has 40° rotation

            // Wait 11: Stay at position for final shooting - CRITICAL FIX!
            .addPath(new BezierLine(
                new Pose(67, 80, Math.toRadians(140)),  // ✓ UPDATED to match Path 10
                new Pose(67, 80, Math.toRadians(140))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(140))
            .setConstraints(waitConstraints) // ✓ ADĂUGAT!

            .build();
    }
}

