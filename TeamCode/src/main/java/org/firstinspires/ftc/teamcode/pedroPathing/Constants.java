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
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.2)
            .forwardZeroPowerAcceleration(-27.28265282915111)
            .lateralZeroPowerAcceleration( -59.93593037757394)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.195,0,0.017,0.013))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2,0,0.03,0.02))//0.65
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0,0.00003,0.6,0.015))
            .centripetalScaling(0.002)

            //Manually added
           // .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
           // .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.02, 0))
           // .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.01, 0.6, 0.01));

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
            .xVelocity(78.95277512918308)
            .yVelocity(63.92800638994832);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.62)
            .strafePodX(-3.31)
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