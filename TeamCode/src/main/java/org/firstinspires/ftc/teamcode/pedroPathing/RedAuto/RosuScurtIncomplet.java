package org.firstinspires.ftc.teamcode.pedroPathing.RedAuto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.*;

@Autonomous(name = "RosuScurtIncomplet", group = "Autonomous")
@Configurable // Panels
public class RosuScurtIncomplet extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer = new ElapsedTime();

    // Robot subsystems
    private TelemetryCustom telemetryCustom;
    private GoBildaPinpointDriver pinpoint;
    private Intake intake;
    private Husky husky;
    private Shooter shooter;
    private Mixer mixer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryCustom = new TelemetryCustom(telemetry);

        // Initialize subsystems
        intake = new Intake(telemetryCustom);
        intake.Initialize(hardwareMap);

        husky = new Husky(telemetryCustom);
        husky.Initialize(hardwareMap);

        mixer = new Mixer(telemetryCustom, intake);
        mixer.Initialize(hardwareMap);
        mixer.SetArtifacts(); // Load 3 balls

        // Pass null for robotAllignment - no IMU conflicts!
        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null, false);
        shooter.Initialize(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Create follower and set starting pose
        // Blue: (17.944, 119.108, 54°)
        // Red:  (144 - 17.944, 119.108, 180 - 54) = (126.056, 119.108, 126°)
        ResetAndCalibratePos(new Pose(126.056, 119.108, Math.toRadians(126)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Mixer", "3 balls loaded");
        panelsTelemetry.debug("Battery Voltage", String.format("%.2fV", shooter.GetBatteryVoltage()));
        panelsTelemetry.update(telemetry);
    }

    private void ResetAndCalibratePos(Pose startPose) {
        Pose2D ftcStartPose = PoseConverter.poseToPose2D(
                startPose,
                InvertedFTCCoordinates.INSTANCE
        );

        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(ftcStartPose);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        intake.SetMotorPower(0.8);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing

        // Update subsystems
        shooter.Run();
        if (shooter.IsShootingStateNone())
            mixer.Run();
        husky.Run();

        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooter RPM", shooter.GetVelocityCurrent());
        panelsTelemetry.debug("Balls in Mixer", mixer.GetArtifactCount());
        panelsTelemetry.debug("Is Shooting?", !shooter.IsNotShooting());
        panelsTelemetry.debug("Shooting Allowed?", shooter.GetShootingAllow());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            // Path 1: Move to shooting position
            // Blue: (17.944, 119.108) -> (60, 120) -> (58, 70) @ 54° -> 140°
            // Red:  (126.056, 119.108) -> (84, 120) -> (86, 70) @ 126° -> 40°
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(126.056, 119.108),
                                    new Pose(144 - 60, 120),    // (84, 120)
                                    new Pose(144 - 58, 70)      // (86, 70)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(40))
                    .build();

            // Path 2: Move to pickup area
            // Blue: (58, 70) -> (35, 70) -> (32, 70)
            // Red:  (86, 70) -> (109, 70) -> (112, 70)
            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144 - 58, 70),     // (86, 70)
                                    new Pose(144 - 35, 70),     // (109, 70)
                                    new Pose(144 - 32, 70)      // (112, 70)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 3: Return to shooting position
            // Blue: (28, 70) -> (55, 80) reversed @ 0° -> 140°
            // Red:  (116, 70) -> (89, 80) reversed @ 180° -> 40°
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 28, 70),     // (116, 70)
                                    new Pose(144 - 55.000, 80)  // (89, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(40))
                    .setReversed()
                    .build();

            // Path 4: Rotate to shooting angle
            // Blue: (55, 80) @ 140°
            // Red:  (89, 80) @ 40°
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 55.000, 80), // (89, 80)
                                    new Pose(144 - 55.000, 80)  // (89, 80)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(40))
                    .build();

            // Path 5: Final movement after second shooting
            // Blue: (55, 80) -> (45.223, 79.020) @ 140°
            // Red:  (89, 80) -> (98.777, 79.020) @ 40°
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 55.000, 80),     // (89, 80)
                                    new Pose(144 - 45.223, 79.020)  // (98.777, 79.020)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(40))
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Start Path 1: Move to shooting position
                follower.followPath(paths.Path1);
                follower.setMaxPower(0.75);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // At shooting position - prepare to shoot
                    follower.setMaxPower(0.8);
                    panelsTelemetry.debug("Status", "At shoot position");
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for shooter to spin up, then start shooting
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f RPM", currentRPM, targetRPM));

                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.seconds() > 2.0) {
                    panelsTelemetry.debug("Status", "🎯 SHOOTING!");
                    shooter.StartAutoShoot();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait until ALL balls shot - WITH MINIMUM TIME
                panelsTelemetry.debug("Status", "🎯 Shooting...");
                panelsTelemetry.debug("Balls Left", mixer.GetArtifactCount());
                panelsTelemetry.debug("Shooting time", String.format("%.2fs", pathTimer.seconds()));

                // ✅ MINIMUM 2.5 seconds to ensure all 3 balls shoot
                if (pathTimer.seconds() > 2.5 && mixer.IsEmpty() && shooter.IsNotShooting()) {
                    panelsTelemetry.debug("Status", "✅ All balls shot!");
                    follower.followPath(paths.Path2, true);
                    follower.setMaxPower(0.3);
                    setPathState(4);
                }
                break;

            case 4:
                follower.setMaxPower(0.3);

                if (!follower.isBusy()) {
                    // Arrived at pickup position
                    intake.SetPowerMax();
                    panelsTelemetry.debug("Status", "At pickup - waiting for balls");
                    setPathState(5);
                }
                break;

            case 5:
                // Wait 1 second at pickup position for balls to be collected
                panelsTelemetry.debug("Pickup Wait", String.format("%.2fs / 1.0s", pathTimer.seconds()));

                if (pathTimer.seconds() > 1.0) {
                    panelsTelemetry.debug("Status", "Pickup complete - returning");
                    follower.followPath(paths.Path3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    // Back at shooting position - rotate to 40°
                    follower.followPath(paths.Path4, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "At shoot position #2");
                    panelsTelemetry.debug("Balls", mixer.GetArtifactCount());
                    shooter.StartAutoShoot();
                    setPathState(8);
                }
                break;

            case 8:
                // Wait until ALL balls shot - WITH MINIMUM TIME
                panelsTelemetry.debug("Status", "🎯 Shooting...");
                panelsTelemetry.debug("Balls Left", mixer.GetArtifactCount());
                panelsTelemetry.debug("Shooting time", String.format("%.2fs", pathTimer.seconds()));

                // ✅ MINIMUM 2.5 seconds to ensure all balls shoot
                if (pathTimer.seconds() > 2.5 && mixer.IsEmpty() && shooter.IsNotShooting()) {
                    panelsTelemetry.debug("Status", "✅ All balls shot!");
                    intake.SetMotorPower(0.0);
                    follower.followPath(paths.Path5, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "✅✅✅ COMPLETE!");
                    intake.SetMotorPower(0.0);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        intake.SetMotorPower(0.0);
    }
}