package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystem;

import java.util.function.DoubleSupplier;

/** Standalone 4-module swerve drivetrain controller implementing Subsystem interface. */
public class SwerveDriveTrain implements Subsystem {
    private SwervePodv2 frontLeft;
    private SwervePodv2 frontRight;
    private SwervePodv2 backLeft;
    private SwervePodv2 backRight;

    // Module locations relative to robot center.
    private final double flX;
    private final double flY;
    private final double frX;
    private final double frY;
    private final double blX;
    private final double blY;
    private final double brX;
    private final double brY;

    private DoubleSupplier headingRadiansSupplier;

    // Robot dimensions (inches)
    private final double baseWidth;
    private final double baseLength;

    public SwerveDriveTrain(double baseWidth, double baseLength) {
        this.baseWidth = baseWidth;
        this.baseLength = baseLength;

        double halfWidth = baseWidth / 2.0;
        double halfLength = baseLength / 2.0;

        this.flX = -halfWidth;
        this.flY = halfLength;
        this.frX = halfWidth;
        this.frY = halfLength;
        this.blX = -halfWidth;
        this.blY = -halfLength;
        this.brX = halfWidth;
        this.brY = -halfLength;
    }

    @Override
    public void LinkComponents(HardwareMap hwMap) {
        // Get motors from hardware map using your naming convention
        // MotorFS = Fata Stanga (Front Left)
        // MotorFD = Fata Dreapta (Front Right)
        // MotorSS = Spate Stanga (Back Left)
        // MotorSD = Spate Dreapta (Back Right)
        DcMotorEx flDrive = hwMap.get(DcMotorEx.class, "MotorFS");
        DcMotorEx frDrive = hwMap.get(DcMotorEx.class, "MotorFD");
        DcMotorEx blDrive = hwMap.get(DcMotorEx.class, "MotorSS");
        DcMotorEx brDrive = hwMap.get(DcMotorEx.class, "MotorSD");

        // Servo naming: ServoFS, ServoFD, ServoSS, ServoSD
        CRServo flSteer = hwMap.get(CRServo.class, "ServoFS");
        CRServo frSteer = hwMap.get(CRServo.class, "ServoFD");
        CRServo blSteer = hwMap.get(CRServo.class, "ServoSS");
        CRServo brSteer = hwMap.get(CRServo.class, "ServoSD");

        // Encoder naming: EncoderFS, EncoderFD, EncoderSS, EncoderSD
        AnalogInput flEncoder = hwMap.get(AnalogInput.class, "EncoderFS");
        AnalogInput frEncoder = hwMap.get(AnalogInput.class, "EncoderFD");
        AnalogInput blEncoder = hwMap.get(AnalogInput.class, "EncoderSS");
        AnalogInput brEncoder = hwMap.get(AnalogInput.class, "EncoderSD");

        // Create pods
        frontLeft = new SwervePodv2(flDrive, flSteer, flEncoder);
        frontRight = new SwervePodv2(frDrive, frSteer, frEncoder);
        backLeft = new SwervePodv2(blDrive, blSteer, blEncoder);
        backRight = new SwervePodv2(brDrive, brSteer, brEncoder);
    }

    @Override
    public void Initialize(HardwareMap hwMap) {
        // Link hardware components first (creates all pods with motors/servos/encoders)
        LinkComponents(hwMap);

        // Configure each pod (motors, encoders)
        SwervePodv2[] pods = {frontLeft, frontRight, backLeft, backRight};
        DcMotorSimple.Direction[] directions = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
        };
        double[] offsets = {-2.810, -1.438, -0.068, -1.622};

        for (int i = 0; i < pods.length; i++) {
            pods[i].setDriveDirection(directions[i]);
            pods[i].configureEncoder(3.3, false, offsets[i]);
        }
    }

    @Override
    public void Run() {
        read();
        update();
        write();
    }

    public void setHeadingSupplier(DoubleSupplier headingRadiansSupplier) {
        this.headingRadiansSupplier = headingRadiansSupplier;
    }

    public void read() {
        frontLeft.read();
        frontRight.read();
        backLeft.read();
        backRight.read();
    }

    public void update() {
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    public void write() {
        frontLeft.write();
        frontRight.write();
        backLeft.write();
        backRight.write();
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void driveRobotCentric(double x, double y, double omega) {
        setModuleTargetsFromChassisSpeeds(x, y, omega);
    }

    public void driveFieldCentric(double xField, double yField, double omega) {
        if (headingRadiansSupplier == null) {
            driveRobotCentric(xField, yField, omega);
            return;
        }

        double heading = headingRadiansSupplier.getAsDouble();
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double xRobot = xField * cos - yField * sin;
        double yRobot = xField * sin + yField * cos;

        setModuleTargetsFromChassisSpeeds(xRobot, yRobot, omega);
    }

    /**
     * x: strafe right (+)
     * y: forward (+)
     * omega: CCW (+)
     */
    private void setModuleTargetsFromChassisSpeeds(double x, double y, double omega) {
        ModuleVector fl = moduleVector(x, y, omega, flX, flY);
        ModuleVector fr = moduleVector(x, y, omega, frX, frY);
        ModuleVector bl = moduleVector(x, y, omega, blX, blY);
        ModuleVector br = moduleVector(x, y, omega, brX, brY);

        double max = Math.max(1.0, Math.max(Math.max(fl.speed, fr.speed), Math.max(bl.speed, br.speed)));

        frontLeft.setTarget(fl.speed / max, fl.heading);
        frontRight.setTarget(fr.speed / max, fr.heading);
        backLeft.setTarget(bl.speed / max, bl.heading);
        backRight.setTarget(br.speed / max, br.heading);
    }

    private static ModuleVector moduleVector(double vx, double vy, double omega, double rx, double ry) {
        // omega x r in 2D => (-omega * ry, omega * rx)
        double wheelVx = vx - omega * ry;
        double wheelVy = vy + omega * rx;

        double speed = Math.hypot(wheelVx, wheelVy);
        double heading = Math.atan2(wheelVy, wheelVx);
        return new ModuleVector(speed, heading);
    }

    private static class ModuleVector {
        final double speed;
        final double heading;

        ModuleVector(double speed, double heading) {
            this.speed = speed;
            this.heading = heading;
        }
    }
}

