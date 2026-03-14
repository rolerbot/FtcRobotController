
package org.firstinspires.ftc.teamcode.PedroAutoTurret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.TurretPositionControl;

@Autonomous(name = "AS9DoubleGate", group = "Autonomous")
@Configurable // Panels
public class AS9DoubleGate extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private Intake intake;
    private LimeLight limeLight;
    private Shooter shooter;
    private Mixer mixer;
    private TurretPositionControl turret;

    private Paths paths; // Paths defined in the Paths class

    private final double maxShootingTime = 2 ; // seconds
    private final double pickupWaitTime = 0.3; // seconds

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        telemetryCustom = new TelemetryCustom(telemetry);

        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        // Initialize LimeLight for Blue alliance (Pipeline 0 for detection first)
        limeLight = new LimeLight(true, true);
        limeLight.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        shooter = new Shooter(telemetryCustom, mixer, limeLight, false);
        shooter.Initialize(hardwareMap);
        shooter.ForceUpdateShooterF();

        turret = new TurretPositionControl(limeLight, shooter, null);
        turret.Initialize(hardwareMap, true); // Reset encoder at start (Right barrier = 0)
        turret.setUsePinpointFallback(false); // Disable odometry fallback in Auto
        turret.setTargetTicks(0); // Set turret to encoder position 0 (physical start) in init

        mixer.MoveToThreeBalls();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.5, 121.6, Math.toRadians(234)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intake.SetMotorPower(1);
        shooter.StartAutoBoost(); // Optimized spin-up
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.Run();
        limeLight.Run();
        turret.Run();

        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Turret Angle", turret.getCurrentAngle());
        panelsTelemetry.debug("Shooter RPM", shooter.GetVelocityCurrent());
        panelsTelemetry.debug("Limelight ID", limeLight.GetID());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {

            // position to shoot
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.5, 121.6),
                                    new Pose(54, 85)))
                    .setLinearHeadingInterpolation(Math.toRadians(234), Math.toRadians(180))

                    .build();

            // pick up balls
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 85.000),

                                    new Pose(21, 85)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // open gate
            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(21, 85),
                                    new Pose(30, 80),
                                     new Pose(18.5, 76)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            // go to shoot
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.5, 76),

                                    new Pose(54.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            // prepare to pick up baalls
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 85.000),

                                    new Pose(53.225, 57)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // pick up balls
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.225, 57),

                                    new Pose(20, 57)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // go back a bit
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20, 57),

                                    new Pose(50 , 57)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // prpeare to open gate
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50, 57),

                                    new Pose(21, 85)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // open gate
            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(21, 85),
                                    new Pose(30, 80),
                                    new Pose(18.5, 76)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            // got to shoot
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.5, 76),

                                    new Pose(55.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();


            // park
            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55, 85),

                                    new Pose(20, 90)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // 1. Drive to shooting position and Search (Pipeline 0)
                follower.setMaxPower(0.8);
                follower.followPath(paths.Path1, true);
                limeLight.getLimelight().pipelineSwitch(0);
                turret.setTrackingTag(false); // No tracking while driving
                setPathState(1);
                break;

            case 1:
                // 2. Wait to arrive at position
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // 3. Arrived. Check if we saw the tag. If not, wait for it (Pipeline 0).
                if (limeLight.GetID() != 0) {
                    setPathState(3);
                } else if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    limeLight.SkipArtifactDetection();
                    // Timeout: proceed even if tag not seen (relies on last known or fallback)
                    setPathState(3);
                }
                break;

            case 3:
                // 4. START ROTATION to midway point (-45 degrees)
                turret.setTrackingTag(false); // Safety: ensure tracking is OFF
                turret.setTargetAngle(-45); // Manually move to ~229 ticks from Right
                pathTimer.resetTimer();
                setPathState(4);
                break;

            case 4:
                // 5. WAIT for turret to finish its manual rotation
                // Must wait at least 0.4s and check if turret reached target
                shooter.StartBackMotorAuto();
                if (pathTimer.getElapsedTimeSeconds() > 0.7
                        && (turret.isOnTarget() || pathTimer.getElapsedTimeSeconds() > 1)) {
                    limeLight.RelocalizationBlue(); // Switch to Blue Shooting Pipeline (2)
                    //turret.setTrackingTag(true); // Enable Active Tracking now
                    pathTimer.resetTimer();
                    setPathState(5); // Proceed to 1s settle wait
                }
                break;

            case 5:
                // 6. Track for a full 1.0 second to ensure accuracy
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.StartAutoShoot();
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                // 7. Monitor shooting progress
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(paths.Path2, true);
                    // follower.setMaxPower(1);
                    setPathState(100); // Changed to avoid overlap with existing states
                }
                break;

            case 100: // Original case 6
                // Picking up balls (Path 2)
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path3, true);
                        setPathState(7);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 7:
                // Opening gate (Path 3)
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathTimer.resetTimer();
                    setPathState(19);
                }
                break;

            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 1)
                    setPathState(8);

            case 8:
                // Go to shoot after gate (Path 4)
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        shooter.StartAutoShoot();
                        pathTimer.resetTimer();
                        setPathState(9);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 9:
                // Finished shooting, prepare to pick up more balls (Path 5)
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path5, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Moving to pickup position (Path 6)
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.followPath(paths.Path6, true);
                    setPathState(11);
                }
                break;

            case 11:
                // Picking up balls (Path 6 wait)
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > pickupWaitTime) {
                        follower.followPath(paths.Path7, true);
                        setPathState(12);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 12:
                // Go back a bit (Path 7)
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    setPathState(13);
                }
                break;


            case 13:
                //prepare to open gate
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    setPathState(14);
                }
                break;

            case 14:
                // Opening gate (Path 3)
                mixer.Run();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    pathTimer.resetTimer();
                    setPathState(18);
                }
                break;

            case 18:
                if(pathTimer.getElapsedTimeSeconds() > 1)
                    setPathState(15);

                // go to shoot
            case 15:
                // Shooting position again (Path 8)
                mixer.Run();
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.3) {
                        shooter.StartAutoShoot();
                        pathTimer.resetTimer();
                        setPathState(16);
                    }
                } else {
                    pathTimer.resetTimer();
                }
                break;

            case 16:
                //finish shooting
                if (mixer.IsEmpty() || pathTimer.getElapsedTimeSeconds() > maxShootingTime) {
                    follower.followPath(paths.Path11, true);
                    setPathState(17);
                }
                break;

            case 17:
                // Parking
                mixer.Run();
                if (!follower.isBusy()) {
                    intake.SetMotorPower(0.0);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        intake.SetMotorPower(0.0);
        if (shooter != null)
            shooter.StopShooterMotors();
    }

}
