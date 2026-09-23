package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "UsingOdometry", group = "Linear Opmode")
public class UsingOdometry extends LinearOpMode {
    private GoBildaPinpointDriver odometry;
    private void Initialize() {
       odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
       odometry.initialize();
       odometry.setOffsets(0, 0, DistanceUnit.CM);
       odometry.setHeading(0, AngleUnit.DEGREES);
       odometry.update();
    }

    private void DisplayOdometryData() {
        // Display telemetry data for odometry readings
        telemetry.addData("X Position", odometry.getEncoderX());
        telemetry.addData("Y Position", odometry.getEncoderY());
        telemetry.addData("Heading", odometry.getHeading(AngleUnit.DEGREES));
    }

    public void runOpMode() {
        Initialize();
        waitForStart();

        while (opModeIsActive()) {
            DisplayOdometryData();
            telemetry.update();
        }
        telemetry.update();
    }
}
