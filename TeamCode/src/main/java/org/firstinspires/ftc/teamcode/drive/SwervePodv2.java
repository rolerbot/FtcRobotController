package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Standalone swerve pod controller (drive motor + steering CR servo + analog angle sensor). */
public class SwervePodv2 {
    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AnalogInput steerEncoder;

    private double encoderMaxVoltage = 3.3;
    private boolean encoderInverted = false;
    private double encoderOffsetVolts = 0.0;

    private double currentHeadingRad = 0.0;
    private double prevWrappedHeadingRad = 0.0;
    private double unwrappedHeadingRad = 0.0;

    private double targetHeadingRad = 0.0;
    private double targetDrivePower = 0.0;

    private double appliedDrivePower = 0.0;
    private double appliedSteerPower = 0.0;

    // Pod tuning defaults (kept close to your old behavior)
    private double kP = 0.6;
    private double kI = 0.0;
    private double kD = 0.0002;
    private double headingToleranceRad = Math.toRadians(1.25);
    private double angleDeadzoneRad = Math.toRadians(60.0);
    private double powerDeadzone = 0.01;
    private double motorSlewStep = 0.2;
    private double maxMotorPower = 1.0;
    private double maxServoPower = 1.0;

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastUpdateNs = 0L;

    private boolean headingOverride = false;

    public SwervePodv2(DcMotorEx driveMotor, CRServo steerServo, AnalogInput steerEncoder) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.steerEncoder = steerEncoder;

        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        steerServo.setPower(0.0);
    }

    public void configureEncoder(double maxVoltage, boolean inverted, double offsetVolts) {
        this.encoderMaxVoltage = maxVoltage;
        this.encoderInverted = inverted;
        this.encoderOffsetVolts = offsetVolts;
    }

    public void setDriveDirection(DcMotor.Direction direction) {
        driveMotor.setDirection(direction);
    }

    public void setTarget(double drivePower, double headingRad) {
        this.targetDrivePower = clamp(drivePower, -maxMotorPower, maxMotorPower);
        this.targetHeadingRad = headingRad;
    }

    public void setTargetHeading(double headingRad) {
        this.targetHeadingRad = headingRad;
    }

    public void setTargetDrivePower(double power) {
        this.targetDrivePower = clamp(power, -maxMotorPower, maxMotorPower);
    }

    public void setHeadingOverride(boolean enabled) {
        this.headingOverride = enabled;
    }

    public void setManualSteerPower(double power) {
        headingOverride = Math.abs(power) > 1e-6;
        steerServo.setPower(clamp(power, -maxServoPower, maxServoPower));
        appliedSteerPower = clamp(power, -maxServoPower, maxServoPower);
    }

    public void setManualDrivePower(double power) {
        driveMotor.setPower(clamp(power, -maxMotorPower, maxMotorPower));
        appliedDrivePower = clamp(power, -maxMotorPower, maxMotorPower);
    }

    public void read() {
        double wrapped = readWrappedHeading();
        double delta = wrapped - prevWrappedHeadingRad;
        if (delta > Math.PI) delta -= 2.0 * Math.PI;
        if (delta < -Math.PI) delta += 2.0 * Math.PI;
        unwrappedHeadingRad += delta;
        prevWrappedHeadingRad = wrapped;

        currentHeadingRad = wrapAngle(unwrappedHeadingRad);
    }

    public void update() {
        double error = wrapAngle(targetHeadingRad - currentHeadingRad);
        double localDriveTarget = targetDrivePower;

        // Rotate shortest way: if > 90deg, flip module heading and wheel direction.
        if (Math.abs(error) > Math.PI * 0.5) {
            targetHeadingRad = wrapAngle(targetHeadingRad - Math.PI);
            error = wrapAngle(targetHeadingRad - currentHeadingRad);
            localDriveTarget *= -1.0;
        }

        if (Math.abs(localDriveTarget) < powerDeadzone || Math.abs(error) > angleDeadzoneRad) {
            localDriveTarget = 0.0;
        }

        if (Math.abs(localDriveTarget - appliedDrivePower) > motorSlewStep) {
            localDriveTarget = appliedDrivePower + Math.signum(localDriveTarget - appliedDrivePower) * motorSlewStep;
        }

        targetDrivePower = localDriveTarget;

        double servoCommand = 0.0;
        if (Math.abs(error) > headingToleranceRad) {
            servoCommand = calculatePid(error);
            servoCommand = clamp(servoCommand, -maxServoPower, maxServoPower);
            if (Math.abs(servoCommand) < powerDeadzone) servoCommand = 0.0;
        } else {
            integral = 0.0;
            lastError = 0.0;
        }

        if (!headingOverride) {
            appliedSteerPower = servoCommand;
        }
    }

    public void write() {
        appliedDrivePower = clamp(targetDrivePower, -maxMotorPower, maxMotorPower);
        driveMotor.setPower(appliedDrivePower);

        if (!headingOverride) {
            steerServo.setPower(appliedSteerPower);
        }
    }

    public void stop() {
        targetDrivePower = 0.0;
        appliedDrivePower = 0.0;
        appliedSteerPower = 0.0;
        driveMotor.setPower(0.0);
        steerServo.setPower(0.0);
    }

    public double getCurrentHeadingRad() {
        return currentHeadingRad;
    }

    public double getTargetHeadingRad() {
        return targetHeadingRad;
    }

    public double getAppliedDrivePower() {
        return appliedDrivePower;
    }

    public double getAppliedSteerPower() {
        return appliedSteerPower;
    }

    public void setPid(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    private double calculatePid(double error) {
        long now = System.nanoTime();
        double dt;
        if (lastUpdateNs == 0L) {
            dt = 0.02;
        } else {
            dt = (now - lastUpdateNs) / 1e9;
            if (dt <= 1e-6) dt = 1e-6;
        }
        lastUpdateNs = now;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    private double readWrappedHeading() {
        double volts = steerEncoder.getVoltage();
        if (encoderInverted) volts = encoderMaxVoltage - volts;
        volts += encoderOffsetVolts;

        while (volts < 0.0) volts += encoderMaxVoltage;
        while (volts >= encoderMaxVoltage) volts -= encoderMaxVoltage;

        return (volts / encoderMaxVoltage) * (2.0 * Math.PI);
    }

    private static double wrapAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

