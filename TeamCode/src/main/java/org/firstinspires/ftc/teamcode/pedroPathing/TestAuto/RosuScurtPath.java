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

@Autonomous(name = "RosuScurtTestPath", group = "Autonomous")
@Configurable
public class RosuScurtPath extends OpMode {
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
        follower.setStartingPose(new Pose(125.992, 121.402, Math.toRadians(126)));

        paths = new Paths(follower);

        pathState = 0; // Initialize state to 0

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start the first path when autonomous begins
        pathState = 1;
        follower.followPath(paths.Path1);
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.992, 121.402),
                                    new Pose(101.382, 126.562),
                                    new Pose(89.466, 98.151)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(70))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.466, 98.151),
                                    new Pose(87.912, 84.215),
                                    new Pose(100.821, 83.873)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.821, 83.873),
                                    new Pose(128.378, 83.625)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1: // Following Path1
                if (!follower.isBusy()) {
                    // Path1 complete, start Path2
                    follower.followPath(paths.Path2);
                    pathState = 2;
                }
                break;

            case 2: // Following Path2
                if (!follower.isBusy()) {
                    // Path2 complete, start Path3
                    follower.followPath(paths.Path3);
                    pathState = 3;
                }
                break;

            case 3: // Following Path3
                if (!follower.isBusy()) {
                    // All paths complete
                    pathState = 4;
                }
                break;

            case 4: // Done
                // All paths finished, autonomous complete
                break;
        }

        return pathState;
    }
}