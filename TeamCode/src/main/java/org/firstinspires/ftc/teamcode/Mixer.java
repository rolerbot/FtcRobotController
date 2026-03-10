package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Mixer implements Subsystem {

    public Servo ServoMixer1 = null;
    public Servo ServoMixer2 = null;
    public DcMotorEx MotorMixer = null;
    private TelemetryCustom logger;
    private final Intake intake;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timerResetEnc = new ElapsedTime();
    private boolean startTimer = false;
    private boolean resetEnc = true;
    private boolean isRunning = false;
    private boolean waitForBall = false;
    private boolean startedReset = false;
    private boolean needsInitialReset = true;
    private double offsetPosition = 0.09028;
    private final double initialPosition = 0.0827 + 2 * offsetPosition;
    // 0.0257
    private double currentPosition = initialPosition;
    boolean waitForReset = false;
    public boolean stopDetection = false;
    private int countPurple = 0;
    private int countGreen = 0;
    ColorSensor cSensor;
    public Color[] artifacte = { Color.None, Color.None, Color.None };
    int artifactCount = 0;

    public Mixer(TelemetryCustom lg, Intake intake) {
        this.intake = intake;
        this.logger = lg;
    }

    public void LinkComponents(HardwareMap hardwareMap) {

        cSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        ServoMixer1 = hardwareMap.get(Servo.class, "ServoMixer1");
        ServoMixer2 = hardwareMap.get(Servo.class, "ServoMixer2");
        MotorMixer = hardwareMap.get(DcMotorEx.class, "MotorIN");
    }

    public void Initialize(HardwareMap hwMap) {
        LinkComponents(hwMap);
        ServoMixer1.setDirection(Servo.Direction.FORWARD);
        ServoMixer2.setDirection(Servo.Direction.FORWARD);
        ServoMixer1.setPosition(initialPosition);
        ServoMixer2.setPosition(initialPosition);

        MotorMixer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorMixer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorMixer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void Reset() {
        ServoMixer1.setPosition(initialPosition);
        ServoMixer2.setPosition(initialPosition);
        currentPosition = initialPosition;
    }

    public void StartTimer() {
        runtime.reset();
        runtime.startTime();
    }

    public void SetMinimPos() {
        ServoMixer1.setPosition(0);
        ServoMixer2.setPosition(0);
        currentPosition = 0;
    }

    public void SetMaximPos() {
        ServoMixer1.setPosition(1);
        ServoMixer2.setPosition(1);
        currentPosition = 1;
    }

    public void ResetEncoder() {
        if (needsInitialReset && !startTimer) {
            startTimer = true;
            timerResetEnc.reset();
            ServoMixer1.setPosition(initialPosition);
            ServoMixer2.setPosition(initialPosition);
        }

        if (startTimer && timerResetEnc.seconds() > 0.35) {
            MotorMixer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorMixer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorMixer.setDirection(DcMotorSimple.Direction.FORWARD);

            needsInitialReset = false;
            startTimer = false;
        }
    }

    public double GetCurrentPosition() {
        return currentPosition;
    }

    public int GetEncoderPosition() {
        return MotorMixer.getCurrentPosition();
    }

    public boolean IsMoving() {
        return isRunning || needsInitialReset || startedReset || waitForReset;
    }

    /// Rotates the Mixer 60 degrees to a side, depending on servo limits.
    void NextPosition() {
        double newPosition = this.GetCurrentPosition() + 2 * offsetPosition;
        ServoMixer1.setPosition(newPosition);
        ServoMixer2.setPosition(newPosition);
        currentPosition = newPosition;
    }

    public void ResetServoPosition() {
        ServoMixer1.setPosition(initialPosition);
        ServoMixer2.setPosition(initialPosition);
        currentPosition = initialPosition;
        artifactCount = 0;
        isRunning = false;
        waitForBall = false;
        runtime.reset();
        if (offsetPosition < 0)
            offsetPosition *= (-1);
        for (int i = 0; i <= 2; i++)
            artifacte[i] = Color.None;
    }

    void ArtifacteIndx() {
        // Don't run if we're busy or if Shooter has taken priority

        if (artifactCount < 0)
            artifactCount = 0; // Absolute safety check

        if (!intake.IsStopped() && !isRunning && !waitForBall && artifactCount < 3) {
            StartTimer();
            Color detectedColor = Culoare(cSensor);

            if (detectedColor != Color.None) {
                artifacte[artifactCount++] = detectedColor;
                CalculateFrequency();
                isRunning = true;
                logger.Log("Detected Color", Utils.ColorToString(detectedColor));
                logger.Log("Nr. Bile in mixer", artifactCount);
            }
        } else if (isRunning && !waitForBall && GetTimerElapsed() < 0.2) {
            waitForBall = true;
            if (artifactCount < 3) {
                NextPosition();
            }
        } else if (isRunning && waitForBall && GetTimerElapsed() >= 0.6) {
            isRunning = false;
            waitForBall = false;
            CalculateFrequency();
        }
    }

    private void CalculateFrequency() {
        countPurple = 0;
        countGreen = 0;
        for (Color col : artifacte) {
            if (col == Color.Purple)
                countPurple++;
            else if (col == Color.Green)
                countGreen++;
        }
        logger.Log("Nr. bile mov", countPurple);
        logger.Log("Nr. bile verzi", countGreen);
    }

    public int GetColorPosition(Color color) {
        logger.Log("Searching for", Utils.ColorToString(color));
        for (int i = 0; i < 3; i++) {
            logger.Log(String.format("Slot %d", i), Utils.ColorToString(artifacte[i]));
            if (artifacte[i] != Color.None && artifacte[i] == color) {
                logger.Log("Found at position", i);
                return i;
            }
        }
        logger.Log("Color not found", -1);
        return -1;
    }

    public int GetCountGreen() {
        return countGreen;
    }

    public int GetCountPurple() {
        return countPurple;
    }

    private Color Culoare(ColorSensor cSensor) {
        int red = cSensor.red();
        int green = cSensor.green();
        int blue = cSensor.blue();

        if (green > 700 && green > red * 1.5 && green > blue * 1.3) {
            // Green must be dominant and significantly higher than red and blue
            return Color.Green;
        }

        if (blue > 650 && blue > red * 1.1 && blue > green * 1.1) {
            // Blue must be dominant for purple
            return Color.Purple;
        }

        // Background/nothing detection - all values roughly equal and low-ish
        if (Math.abs(red - green) < 100 && Math.abs(red - blue) < 100 && Math.abs(green - blue) < 100) {
            return Color.None;
        }

        return Color.None;
    }

    public boolean IsEmpty() {
        return artifactCount == 0;
    }

    public void RemoveArtifact(int position) {
        if (position >= 0 && position < artifacte.length) {
            if (artifacte[position] != Color.None) {
                artifacte[position] = Color.None;
                artifactCount--;
                if (artifactCount < 0)
                    artifactCount = 0;
                CalculateFrequency();
            }
        }
    }

    public void SetArtifacts() {
        artifacte[0] = Color.Green;
        artifacte[1] = Color.Purple;
        artifacte[2] = Color.Purple;
        artifactCount = 3;
        CalculateFrequency();
    }

    public void SetPosition(double pos) {
        if (pos < 0.0 || pos > 1.0)
            return;
        ServoMixer1.setPosition(pos);
        ServoMixer2.setPosition(pos);
    }

    public double GetTimerElapsed() {
        return runtime.seconds();
    }

    public int GetArtifactCount() {
        return artifactCount;
    }

    public void Run() {
        ArtifacteIndx();
        ResetEncoder();
    }
}