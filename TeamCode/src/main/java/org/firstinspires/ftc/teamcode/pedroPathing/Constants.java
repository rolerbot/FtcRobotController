package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    // ROBOT DIMENSIONS (in inches) - IMPORTANT for path planning!
    // Measure your robot and update these values
    public static final double ROBOT_LENGTH = 18.0;  // Front to back (inches)
    public static final double ROBOT_WIDTH = 18.0;   // Left to right (inches)
    public static final double ROBOT_DIAGONAL = Math.sqrt(ROBOT_LENGTH * ROBOT_LENGTH + ROBOT_WIDTH * ROBOT_WIDTH);

    // Calculated offsets from center to edges
    public static final double HALF_LENGTH = ROBOT_LENGTH / 2.0;  // 9 inches
    public static final double HALF_WIDTH = ROBOT_WIDTH / 2.0;    // 9 inches

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.15)
            .forwardZeroPowerAcceleration(-29)
            .lateralZeroPowerAcceleration(-52)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.195,0,0.017,0.013))
            .headingPIDFCoefficients(new PIDFCoefficients(0.65,0,0.015,0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0,0.00003,0.6,0.015));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("MotorFD")
            .rightRearMotorName("MotorSD")
            .leftRearMotorName("MotorSS")
            .leftFrontMotorName("MotorFS")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(63.0)
            .yVelocity(64.0);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.65)
            .strafePodX(-3.62)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
