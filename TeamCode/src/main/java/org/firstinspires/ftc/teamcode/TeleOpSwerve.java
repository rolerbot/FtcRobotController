package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SwerveDriveTrain;

@TeleOp(name = "TeleOpSwerve", group = "Drive")
public class TeleOpSwerve extends OpMode {
    private SwerveDriveTrain swerve;

    @Override
    public void init() {
        // Robot dimensions (inches) - adjust to your chassis
        // Assuming 13.25 x 12.50 inches
        double baseWidth = 13.25;
        double baseLength = 12.50;

        // Create drivetrain
        swerve = new SwerveDriveTrain(baseWidth, baseLength);

        // Initialize components
        swerve.Initialize(hardwareMap);

        telemetry.addLine("Swerve TeleOp initialized");
        telemetry.addLine("Left Stick: strafe/forward | Right Stick: rotate");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get gamepad input
        double strafe = -gamepad1.left_stick_x;   // left stick x (right is positive)
        double forward = gamepad1.left_stick_y;   // left stick y (forward is positive)
        double rotate = -gamepad1.right_stick_x;  // right stick x (CCW is positive)

        // Apply deadzone
        strafe = deadzone(strafe, 0.05);
        forward = deadzone(forward, 0.05);
        rotate = deadzone(rotate, 0.05);

        // Drive robot-centric (can add field-centric later with Pinpoint heading)
        swerve.driveRobotCentric(strafe, forward, rotate);

        // Run the drivetrain (read -> update -> write)
        swerve.Run();

        // Telemetry
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Forward", forward);
        telemetry.addData("Rotate", rotate);
        telemetry.update();
    }

    @Override
    public void stop() {
        swerve.stop();
    }

    private double deadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }
}

