package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.PinpointBlocksDriver.GoBildaPinpointDriver;

@TeleOp(name = "Pinpoint Field Display")
public class TeleOpTest extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;
    FtcDashboard dashboard;
    Drivetrain drivetrain;
    GamepadEx ct1, ct2;

    @Override
    public void runOpMode() {
        // Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure your Pinpoint (adjust these values for your setup)
        pinpoint.setOffsets(-3.62, -6.65); // X and Y offsets in mm
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        dashboard = FtcDashboard.getInstance();

        ct1 = new GamepadEx(gamepad1);
        ct2 = new GamepadEx(gamepad2);

        drivetrain = new Drivetrain(ct1, ct2);
        drivetrain.Initialize(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Pinpoint position
            pinpoint.update();
            drivetrain.Run();
            // Get robot pose
            Pose2D pos = pinpoint.getPosition();
            double x = pos.getX(DistanceUnit.INCH);
            double y = pos.getY(DistanceUnit.INCH);
            double heading = pos.getHeading(AngleUnit.RADIANS);

            // Create telemetry packet
            TelemetryPacket packet = new TelemetryPacket();

            // Draw robot like in the tuner - rectangle with direction indicator
            double robotWidth = 18;  // Width of robot in inches
            double robotHeight = 18; // Height of robot in inches

            // Calculate corner points of the robot rectangle
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            double halfWidth = robotWidth / 2;
            double halfHeight = robotHeight / 2;

            // Robot corners (rotating rectangle around center)
            double[] xCorners = new double[4];
            double[] yCorners = new double[4];

            // Front-left corner
            xCorners[0] = x + (-halfWidth * cos - halfHeight * sin);
            yCorners[0] = y + (-halfWidth * sin + halfHeight * cos);

            // Front-right corner
            xCorners[1] = x + (halfWidth * cos - halfHeight * sin);
            yCorners[1] = y + (halfWidth * sin + halfHeight * cos);

            // Back-right corner
            xCorners[2] = x + (halfWidth * cos + halfHeight * sin);
            yCorners[2] = y + (halfWidth * sin - halfHeight * cos);

            // Back-left corner
            xCorners[3] = x + (-halfWidth * cos + halfHeight * sin);
            yCorners[3] = y + (-halfWidth * sin - halfHeight * cos);

            // Draw robot body as rectangle
            packet.fieldOverlay()
                    .setStroke("#4CAF50")  // Green outline
                    .setStrokeWidth(1)
                    .strokePolygon(xCorners, yCorners);

            // Draw center point (shows robot center of rotation)
            packet.fieldOverlay()
                    .setFill("#FF5722")  // Red center point
                    .fillCircle(x, y, 2);

            // Draw front direction indicator (longer line showing front of robot)
            double frontLineLength = robotHeight / 2 + 8;  // Extends past robot
            packet.fieldOverlay()
                    .setStroke("#2196F3")  // Blue front indicator
                    .setStrokeWidth(2)
                    .strokeLine(x, y,
                            x + frontLineLength * Math.cos(heading),
                            y + frontLineLength * Math.sin(heading));

            // Send to dashboard
            dashboard.sendTelemetryPacket(packet);

            // Also send telemetry data
            telemetry.addData("X (inches)", "%.2f", x);
            telemetry.addData("Y (inches)", "%.2f", y);
            telemetry.addData("Heading (degrees)", "%.2f", Math.toDegrees(heading));
            telemetry.addData("", "");
            telemetry.addData("Tip", "Rotate robot to check if center stays in place");
            telemetry.update();

            sleep(20);  // Update rate
        }
    }
}