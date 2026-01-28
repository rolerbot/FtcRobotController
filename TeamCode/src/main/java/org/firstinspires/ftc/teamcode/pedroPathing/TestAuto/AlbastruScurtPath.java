package org.firstinspires.ftc.teamcode.pedroPathing.TestAuto;
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

// ADD THESE FOR DASHBOARD VISUALIZATION
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Autonomous(name = "AlbastruScurtPathPedro", group = "Autonomous")
@Configurable
public class AlbastruScurtPath extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private FtcDashboard dashboard; // For field visualization

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        dashboard = FtcDashboard.getInstance();

        follower = Constants.createFollower(hardwareMap);
        // Mirrored starting pose (mirrored X coordinate and heading)
        follower.setStartingPose(new Pose(144 - 117.960, 132.494, Math.toRadians(180 - 126.5)));

        paths = new Paths(follower);

        pathState = 0; // Initialize state to 0

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start the first path when autonomous begins
        pathState = 1;
        follower.followPath(paths.Path1a);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        // Visualize robot on dashboard
        drawRobotOnDashboard();

        // Log values
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    private void drawRobotOnDashboard() {
        TelemetryPacket packet = new TelemetryPacket();

        Pose pose = follower.getPose();
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        // Draw robot (same visualization as your TeleOp)
        double robotWidth = 18;
        double robotHeight = 18;

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double halfWidth = robotWidth / 2;
        double halfHeight = robotHeight / 2;

        double[] xCorners = new double[4];
        double[] yCorners = new double[4];

        xCorners[0] = x + (-halfWidth * cos - halfHeight * sin);
        yCorners[0] = y + (-halfWidth * sin + halfHeight * cos);

        xCorners[1] = x + (halfWidth * cos - halfHeight * sin);
        yCorners[1] = y + (halfWidth * sin + halfHeight * cos);

        xCorners[2] = x + (halfWidth * cos + halfHeight * sin);
        yCorners[2] = y + (halfWidth * sin - halfHeight * cos);

        xCorners[3] = x + (-halfWidth * cos + halfHeight * sin);
        yCorners[3] = y + (-halfWidth * sin - halfHeight * cos);

        // Draw robot body
        packet.fieldOverlay()
                .setStroke("#4CAF50")
                .setStrokeWidth(1)
                .strokePolygon(xCorners, yCorners);

        // Draw center point
        packet.fieldOverlay()
                .setFill("#FF5722")
                .fillCircle(x, y, 2);

        // Draw front indicator
        double frontLineLength = robotHeight / 2 + 8;
        packet.fieldOverlay()
                .setStroke("#2196F3")
                .setStrokeWidth(2)
                .strokeLine(x, y,
                        x + frontLineLength * Math.cos(heading),
                        y + frontLineLength * Math.sin(heading));

        // Draw the path being followed (optional but helpful)
        packet.fieldOverlay()
                .setStroke("#FFA500")
                .setStrokeWidth(1);

        dashboard.sendTelemetryPacket(packet);
    }

    public static class Paths {
        public PathChain Path1a;  // Curve with constant heading
        public PathChain Path1b;  // Rotate in place
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            // Path1a: Follow the curve WITHOUT rotating (mirrored)
            Path1a = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144 - 117.960, 132.494),
                                    new Pose(144 - 90.291, 122.347),
                                    new Pose(144 - 91.952, 98.534)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180 - 126.5))  // Keep heading constant (mirrored)
                    .build();

            // Path1b: Rotate in place from mirrored 126.5° to mirrored 68°
            Path1b = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 91.952, 98.534),  // Same position
                                    new Pose(144 - 91.952, 98.534)   // Same position (no movement)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 126.5), Math.toRadians(180 - 68))  // Only rotate (mirrored)
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 91.952, 98.534),
                                    new Pose(144 - 95.570, 83.402)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 68), Math.toRadians(180 - 0))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 95.570, 83.402),
                                    new Pose(144 - 112, 83.661)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1: // Following Path1a (curve)
                if (!follower.isBusy()) {
                    // Path1a complete, start Path1b (rotate in place)
                    follower.followPath(paths.Path1b);
                    pathState = 2;
                }
                break;

            case 2: // Following Path1b (rotate in place)
                if (!follower.isBusy()) {
                    // Path1b complete, start Path2
                    follower.setMaxPower(0.7);
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;

            case 3: // Following Path2
                if (!follower.isBusy()) {
                    // Path2 complete, start Path3
                    follower.followPath(paths.Path3);
                    pathState = 4;
                }
                break;

            case 4: // Following Path3
                if (!follower.isBusy()) {
                    // All paths complete
                    pathState = 5;
                }
                break;

            case 5: // Done
                // All paths finished, autonomous complete
                break;
        }

        return pathState;
    }
}