package org.firstinspires.ftc.teamcode.pedroPathing;
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


@Autonomous(name = "RosuLung", group = "Autonomous")
@Configurable // Panels
public class RosuLung extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer = new ElapsedTime();
    private final double pickupWaitTime = 1.0; // 1 second wait at pickup positions

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
        shooter = new Shooter(telemetryCustom, mixer, intake, husky, null);
        shooter.Initialize(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Create follower and set starting pose
        ResetAndCalibratePos(new Pose(81.000, 9.000, Math.toRadians(0)));

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
        if (!shooter.GetShootingAllow())
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

        public Paths(Follower follower) {
            // Path 1: Move up and rotate to 90°
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(81.000, 9.000),
                                    new Pose(89.484, 21.970),
                                    new Pose(82.582, 41.355)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            // Path 2: Move back at 68° to shooting position
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.582, 41.355),
                                    new Pose(83.000, 9.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                    .build();

            // Path 3: Rotate to 0° and move to pickup
            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(83.000, 9.000),
                                    new Pose(83.191, 20),
                                    new Pose(87, 34.924)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                    .build();

            // Path 4: Continue to pickup position
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87, 34.924),
                                    new Pose(130, 35.526)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 5: Return to shooting position and rotate to 68°
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130, 35.526),
                                    new Pose(83.000, 9.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                    .build();

            // Path 6: Move to second pickup area (rotate to 0°)
            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(83.000, 9.000),
                                    new Pose(90, 14.181),
                                    new Pose(100, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100, 10),
                                    new Pose(130, 10)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130, 10),
                                    new Pose(112.884, 10),
                                    new Pose(83.000, 9.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                    .build();

            // Path 9: Move forward a bit after final shooting
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.000, 9.000),
                                    new Pose(87.434, 20.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(paths.Path1);
                follower.setMaxPower(0.75);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    follower.setMaxPower(0.8);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    panelsTelemetry.debug("Status", "At shoot position");
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for shooter to spin up, then start shooting (EXACTLY LIKE BLUE)
                double currentRPM = shooter.GetVelocityCurrent();
                double targetRPM = shooter.GetVelocityTarget();

                panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f RPM", currentRPM, targetRPM));

                if (Math.abs(currentRPM - targetRPM) < 100 || pathTimer.seconds() > 2.0) {
                    panelsTelemetry.debug("Status", "🎯 SHOOTING!");
                    shooter.StartAutoShoot();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait until ALL balls shot (EXACTLY LIKE BLUE)
                if (mixer.IsEmpty() && shooter.IsNotShooting()) {
                    panelsTelemetry.debug("Status", "✅ All balls shot!");
                    follower.followPath(paths.Path3, true);
                    follower.setMaxPower(1);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.Path4, true);
                    setPathState(6);
                }
                break;

            case 6:
                follower.setMaxPower(0.3);

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    setPathState(7); // Move to 1-second wait state
                }
                break;

            case 7:
                // Wait 1 second at first pickup position (like blue's pickupWaitTime)
                if (pathTimer.seconds() > pickupWaitTime) {
                    follower.followPath(paths.Path5, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back at shoot #2");
                    panelsTelemetry.debug("Balls", mixer.GetArtifactCount());
                    shooter.StartAutoShoot();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait until ALL balls shot
                if (mixer.IsEmpty() && shooter.IsNotShooting()) {
                    panelsTelemetry.debug("Status", "✅ All balls shot!");
                    follower.followPath(paths.Path6, true);
                    follower.setMaxPower(1);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    intake.SetPowerMax();
                    follower.setMaxPower(0.3);
                    follower.followPath(paths.Path7, true);
                    setPathState(11);
                }
                break;

            case 11:
                follower.setMaxPower(0.3);

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);
                    setPathState(12); // Move to 1-second wait state
                }
                break;

            case 12:
                // Wait 1 second at second pickup position
                if (pathTimer.seconds() > pickupWaitTime) {
                    follower.followPath(paths.Path8, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Back - final shooting");
                    panelsTelemetry.debug("Balls", mixer.GetArtifactCount());
                    shooter.StartAutoShoot();
                    setPathState(14);
                }
                break;

            case 14:
                // Wait until ALL balls shot
                if (mixer.IsEmpty() && shooter.IsNotShooting()) {
                    panelsTelemetry.debug("Status", "✅ All balls shot!");
                    intake.SetMotorPower(0.0);
                    follower.followPath(paths.Path9, true);
                    setPathState(15);
                }
                break;

            case 15:
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