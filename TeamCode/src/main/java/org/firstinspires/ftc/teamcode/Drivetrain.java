package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain implements Subsystem {
    private DcMotorEx MotorFS = null;
    /// Fata stanga
    private DcMotorEx MotorFD = null;
    /// Fata dreapta
    private DcMotorEx MotorSS = null;
    /// Spate stanga
    private DcMotorEx MotorSD = null;
    private ButtonReader Viteza;
    private final GamepadEx ct1, ct2;
    private RobotPinpoint robotPinpoint = null;
    double schimbator = 0.4;// Viteza 0.4
    double[] speeds = new double[4];
    double drive, strafe, twist;
    private double[] headingLockCorrection = { 0, 0, 0, 0 }; // Heading lock corrections
    private double[] positionLockCorrection = { 0, 0, 0, 0 }; // Position lock corrections (takes priority)

    // Active Brake Variables
    private double targetX, targetY, targetH;
    private double currentX, currentY, currentH;
    private boolean isLocked = false;

    public Drivetrain(GamepadEx ct1, GamepadEx ct2) {
        this.ct1 = ct1;
        this.ct2 = ct2;
    }

    public Drivetrain(GamepadEx ct1, GamepadEx ct2, RobotPinpoint robotPinpoint) {
        this.ct1 = ct1;
        this.ct2 = ct2;
        this.robotPinpoint = robotPinpoint;
    }

    public void LinkComponents(HardwareMap hardwareMap) {
        MotorFS = hardwareMap.get(DcMotorEx.class, "MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class, "MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class, "MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class, "MotorSD");
    }

    public void Initialize(HardwareMap hwMap) {
        LinkComponents(hwMap);
        Viteza = new ButtonReader(ct1, GamepadKeys.Button.B);

        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFD.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSD.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void Run() {
        Viteza.readValue();
        if (Viteza.wasJustPressed()) {
            schimbator = 1.4 - schimbator;
            // telemetry.addData("viteza este", schimbator);
            // telemetry.update();
        }
        drive = -ct1.getLeftY() * schimbator;
        strafe = ct1.getLeftX() * schimbator;
        twist = schimbator * (ct1.gamepad.left_trigger - ct1.gamepad.right_trigger);

        // --- ACTIVE BRAKE SECTION ---
        // Calling it here makes it internal to Run(). 
        // We check (ct1 != null) to ensure we are in TeleOp and not Auto.
        /*
        if (ct1 != null && robotPinpoint != null && Math.abs(drive) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(twist) < 0.05) {
            updatePinpointData(robotPinpoint.GetX(), robotPinpoint.GetY(), robotPinpoint.GetHeading());
            performActiveBrake();
        } else {
            isLocked = false;
        }
        */
        // -----------------------------

        speeds[0] = (drive - strafe + twist); // FS
        speeds[1] = (drive + strafe - twist); // FD
        speeds[2] = (drive + strafe + twist); // SS
        speeds[3] = (drive - strafe - twist); // SD

        // Add position lock corrections (takes PRIORITY - overrides manual if active)
        speeds[0] += positionLockCorrection[0];
        speeds[1] += positionLockCorrection[1];
        speeds[2] += positionLockCorrection[2];
        speeds[3] += positionLockCorrection[3];

        // Add heading lock corrections (only active when robot is stationary)
        speeds[0] += headingLockCorrection[0];
        speeds[1] += headingLockCorrection[1];
        speeds[2] += headingLockCorrection[2];
        speeds[3] += headingLockCorrection[3];

        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++)
            if (max < Math.abs(speeds[i]))
                max = Math.abs(speeds[i]);
        if (max > 1)
            for (int i = 0; i < speeds.length; i++)
                speeds[i] /= max;
        MotorFS.setPower(speeds[0]);
        MotorFD.setPower(speeds[1]);
        MotorSS.setPower(speeds[2]);
        MotorSD.setPower(speeds[3]);
    }

    public void Rotate(double power) {
        MotorFS.setPower(power);
        MotorFD.setPower(-power);
        MotorSS.setPower(power);
        MotorSD.setPower(-power);
    }

    public void MoveForward(double power) {
        MotorFS.setPower(power);
        MotorFD.setPower(power);
        MotorSS.setPower(power);
        MotorSD.setPower(power);
    }

    public void Stop() {
        MotorFS.setPower(0);
        MotorFD.setPower(0);
        MotorSS.setPower(0);
        MotorSD.setPower(0);
    }

    public void ApplyHeadingLockPowers(double[] lockPowers) {
        if (lockPowers == null || lockPowers.length != 4) {
            // Clear heading lock corrections
            headingLockCorrection[0] = 0;
            headingLockCorrection[1] = 0;
            headingLockCorrection[2] = 0;
            headingLockCorrection[3] = 0;
            return;
        }

        // Store heading lock corrections to be applied in Run()
        headingLockCorrection[0] = lockPowers[0]; // Left Front
        headingLockCorrection[1] = lockPowers[1]; // Right Front
        headingLockCorrection[2] = lockPowers[2]; // Left Back
        headingLockCorrection[3] = lockPowers[3]; // Right Back
    }

    public void ApplyMovementPowers(double[] movementPowers) {
        if (movementPowers == null || movementPowers.length != 4) {
            // Clear position lock corrections
            positionLockCorrection[0] = 0;
            positionLockCorrection[1] = 0;
            positionLockCorrection[2] = 0;
            positionLockCorrection[3] = 0;
            return;
        }

        // Store position lock corrections to be applied in Run()
        // Position lock takes PRIORITY over manual controls and heading lock
        positionLockCorrection[0] = movementPowers[0]; // Left Front
        positionLockCorrection[1] = movementPowers[1]; // Right Front
        positionLockCorrection[2] = movementPowers[2]; // Left Back
        positionLockCorrection[3] = movementPowers[3]; // Right Back
    }

    /**
     * Updates the drivetrain with current Pinpoint data for the Active Brake logic.
     */
    public void updatePinpointData(double x, double y, double h) {
        this.currentX = x;
        this.currentY = y;
        this.currentH = h;
    }

    /**
     * Actively resists movement by calculating power needed to return to the last stopped position.
     * This makes the robot "fight" to stay in one place.
     */
    private void performActiveBrake() {
        if (!isLocked) {
            targetX = currentX;
            targetY = currentY;
            targetH = currentH;
            isLocked = true;
        }

        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double errorH = targetH - currentH;

        // Angle normalization (ensure error is between -180 and 180)
        while (errorH > 180) errorH -= 360;
        while (errorH < -180) errorH += 360;

        // Convert field errors to robot-centric errors based on current heading
        double angleRad = -Math.toRadians(currentH);
        double robotErrorX = errorX * Math.cos(angleRad) - errorY * Math.sin(angleRad);
        double robotErrorY = errorX * Math.sin(angleRad) + errorY * Math.cos(angleRad);

        // PID-like Gains (P only for simplicity)
        // These control how hard the robot "fights" back.
        double kP = 0.08;   // Resistance to movement
        double kPH = 0.01;  // Resistance to turning

        drive += robotErrorX * kP;
        strafe += robotErrorY * kP;
        twist += errorH * kPH;
    }
}
