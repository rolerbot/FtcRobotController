
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


@Autonomous(name = "PAS12Gate", group = "Autonomous")
@Configurable // Panels
public class AS12Gate extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

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
        public PathChain Path7;
        public PathChain Path14;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {

            //position to shoot
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(33.476, 134.481),
                                    new Pose(57.369, 124.144),
                                    new Pose(59.080, 112.235)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            //prepare to pick up balls
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.080, 112.235),

                                    new Pose(54.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();


            //pick up balls
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 85.000),

                                    new Pose(15.610, 84.979)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // open gate
            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.610, 84.979),
                                    new Pose(30.992, 73.479),
                                    new Pose(17.529, 72.214)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            // go to shoot
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.529, 72.214),

                                    new Pose(54.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            // prepare to pick up baalls
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 85.000),

                                    new Pose(53.225, 59.615)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            //pick up balls
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.225, 59.615),

                                    new Pose(16.786, 59.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // go back a bit
            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.786, 59.000),

                                    new Pose(25.000, 59.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // got to shoot
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.000, 59.000),

                                    new Pose(55.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // prepare to pick up balls
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.000, 85.000),

                                    new Pose(49.765, 35.610)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            //pick up balls
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.765, 35.610),

                                    new Pose(13.636, 35.471)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            // go to shoot
            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.636, 35.471),

                                    new Pose(55.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            //park
            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.000, 85.000),

                                    new Pose(30.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        return pathState;
    }


}
    