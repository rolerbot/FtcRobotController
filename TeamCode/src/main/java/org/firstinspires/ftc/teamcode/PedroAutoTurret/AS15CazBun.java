
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


@Autonomous(name = "AS15CazBun", group = "Autonomous")
@Configurable // Panels
public class AS15CazBun extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19, 121, Math.toRadians(144)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 121.000),

                                    new Pose(54.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 90.000),
                                    new Pose(57.741, 61.600),
                                    new Pose(42.263, 59.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.263, 59.000),

                                    new Pose(16.116, 59.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.116, 59.000),
                                    new Pose(45.488, 68.825),
                                    new Pose(54.000, 90.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 90.000),
                                    new Pose(33.500, 48.000),
                                    new Pose(17.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.000, 63.000),

                                    new Pose(12.500, 54.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(160))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.500, 54.000),
                                    new Pose(36.000, 55.000),
                                    new Pose(54.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 90.000),
                                    new Pose(33.500, 48.000),
                                    new Pose(17.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.000, 63.000),

                                    new Pose(12.500, 54.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(160))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.500, 54.000),
                                    new Pose(36.000, 55.000),
                                    new Pose(54.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 90.000),

                                    new Pose(42.622, 84.187)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.622, 84.187),

                                    new Pose(16.964, 84.159)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.964, 84.159),

                                    new Pose(54.000, 90.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path15 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 90.000),

                                    new Pose(19.462, 89.044)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }


    public void autonomousPathUpdate() {
        switch (pathState)
        {
            case 0:
                follower.setMaxPower(0.7);
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path1);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    setPathState(6);
                }
                break;
             case 6:
                 if (!follower.isBusy()) {
                     follower.followPath(paths.Path8);
                     setPathState(7);
                 }
                 break;
             case 7:
                 if (!follower.isBusy()) {
                     follower.followPath(paths.Path9);
                     setPathState(8);
                 }
                 break;
             case 8:
                 if (!follower.isBusy()) {
                     follower.followPath(paths.Path10);
                     setPathState(9);
                 }
                 break;
             case 9:
                 if (!follower.isBusy()) {
                     follower.followPath(paths.Path11);
                     setPathState(10);
                 }
                 break;
             case 10:
                 if (!follower.isBusy()) {
                     follower.followPath(paths.Path12);
                     setPathState(11);
                 }
                 break;
             case 11:
                 if (!follower.isBusy()) {
                     follower.followPath(paths.Path13);
                     setPathState(12);
                 }
                 break;
             case 12:
                 if (!follower.isBusy()) {
                     follower.followPath(paths.Path14);
                     setPathState(13);
                 }
                 break;
             case 13:
                 if (!follower.isBusy()) {
                     follower.followPath(paths.Path15);
                     setPathState(14); // End of paths
                 }
                 break;
        }

    }


}
    