package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Mixer implements Subsystem {
    public Servo ServoMixer1 = null;
    public Servo ServoMixer2 = null;
    public DcMotorEx MotorMixer = null;
    private TelemetryCustom logger;
    private final Intake intake;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timerReset = new ElapsedTime();
    private boolean isRunning = false;
    private boolean waitForBall = false;
    private boolean startedReset = false;
    private boolean needsInitialReset = true;
    private double offsetPosition = 0.09028;
    private final double initialPosition = 0.0827 + 2 * offsetPosition;
    //0.0257
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
        MotorMixer.setDirection(DcMotorEx.Direction.FORWARD);
        MotorMixer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Reset();
    }

    public void Reset() {
        ServoMixer1.setPosition(initialPosition);
        ServoMixer2.setPosition(initialPosition);
        MotorMixer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorMixer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        currentPosition = initialPosition;
    }

    public double GetServoPosConstant() {
        return MotorMixer.getCurrentPosition();
    }

    public void PerformReset() {
        if (!startedReset) {
            logger.Log("Auto Reset", "Step 1: Servo Home...");
            ServoMixer1.setPosition(initialPosition);
            ServoMixer2.setPosition(initialPosition);
            timerReset.reset();
            startedReset = true;
            return;
        }

        // Wait precisely 0.4 seconds for servos to settle
        if (timerReset.seconds() < 0.43) {
            return;
        }

        logger.Log("Auto Reset", "Step 2: Encoder Zero...");
        // Reset encoder (Note: only works if encoder is in incremental mode "S")
        MotorMixer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        MotorMixer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reset position tracking
        currentPosition = initialPosition;
        if (offsetPosition < 0)
            offsetPosition *= (-1);

        // Clear all artifact data
        artifactCount = 0;
        for (int i = 0; i <= 2; i++)
            artifacte[i] = Color.None;
        countPurple = 0;
        countGreen = 0;

        // Reset state flags
        isRunning = false;
        waitForBall = false;
        waitForReset = false;

        // Reset timers
        runtime.reset();

        logger.Log("✓ Reset Complete", "Ready to detect balls");
        logger.Log("Encoder Position", MotorMixer.getCurrentPosition());

        needsInitialReset = false; // FINISHED Reset
        startedReset = false;
    }

    public void StartTimer() {
        runtime.reset();
        runtime.startTime();
    }

    public double GetPosMax()
    {
        return initialPosition - 2 * offsetPosition;
    }

    public int GetColorBlue() {
        return cSensor.blue();
    }

    public int GetColorGreen() {
        return cSensor.green();
    }

    public int GetColorRed() {
        return cSensor.red();
    }

    public double GetCurrentPosition() {
        return currentPosition;
    }

    public void ReverseIncrement() {
        offsetPosition *= (-1);
    }

    public void RequestManualReset() {
        needsInitialReset = true;
        startedReset = false;
        waitForReset = false;
    }

    public boolean IsMoving() {
        return isRunning || needsInitialReset || startedReset || waitForReset;
    }

    private void IncrementPosition() {
        if (this.currentPosition + this.offsetPosition > 1.0 || this.currentPosition + this.offsetPosition < -1.0)
            this.offsetPosition *= (-1);
        this.currentPosition += this.offsetPosition;
    }

    /// Rotates the Mixer 60 degrees to a side, depending on servo limits.
    void NextPosition() {
        IncrementPosition();
        ServoMixer1.setPosition(this.GetCurrentPosition());
        ServoMixer2.setPosition(this.GetCurrentPosition());
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
        if (startedReset || waitForReset || stopDetection)
            return;

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
                int index = 0;
                for (Color col : artifacte) {
                    if (col != null) { // Safety check
                        logger.Log(String.format("Bila mixer pozitie %d", index), Utils.ColorToString(col));
                    }
                    index++;
                }
            }
        } else if (isRunning && !waitForBall && GetTimerElapsed() < 0.31) {
            waitForBall = true;
            if (artifactCount < 3) {
                IncrementPosition();
                NextPosition();
            }
        } else if (isRunning && waitForBall && GetTimerElapsed() >= 0.31) {
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

    public void TelemetryColor() {
        logger.Log("Color red", cSensor.red());
        logger.Log("Color blue", cSensor.blue());
        logger.Log("Color green", cSensor.green());
    }

    public int GetFirstAvailablePosition() {
        for (int i = 0; i < artifacte.length; i++) {
            if (artifacte[i] != Color.None)
                return i;
        }
        return -1;
    }

    public int GetCountGreen() {
        return countGreen;
    }

    public int GetCountPurple() {
        return countPurple;
    }

    private Color Culoare2(ColorSensor cSensor) {
        int red = cSensor.red();
        int green = cSensor.green();
        int blue = cSensor.blue();
        int total = red + green + blue;

        // Lower threshold - detect sample presence faster
        if (total < 400) {
            return Color.None;
        }

        // Green detection: green channel dominates
        // Lowered from 1600 to 800 for faster detection
        if (green > red * 1.4 && green > blue * 1.3 && green > 800) {
            return Color.Green;
        }

        // Purple detection: red + blue high, green relatively low
        // Lowered from 900 to 500 for faster detection
        if (red > green && blue > green * 0.6 && red > 500 && green < 1000) {
            return Color.Purple;
        }

        return Color.None;
    }

    private Color Culoare(ColorSensor cSensor) {
        int red = cSensor.red();
        int green = cSensor.green();
        int blue = cSensor.blue();
        int total = red + green + blue;

        // Green detection - based on your ACTUAL readings
        // Green max: R:1000, G:2800, B:2000
        // Green min: R:400, G:763, B:562
        if (green > 700 && green > red * 1.5 && green > blue * 1.3) {
            // Green must be dominant and significantly higher than red and blue
            return Color.Green;
        }

        // Purple detection - based on your ACTUAL readings
        // Purple max: R:1350, G:1450, B:1870
        // Purple min: R:500, G:600, B:700
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

    /*
     * purple:
     * r: 759
     * g: 580
     * b: 406
     * 
     * green:
     * r:727
     * b:400
     * g:590
     */

    private Color Culoare1(ColorSensor cSensor) {
        int red = cSensor.red();
        int green = cSensor.green();
        int blue = cSensor.blue();

        if (green < 300 && red > blue && red > green && blue > green)
            return Color.Purple; // Mov

        if (green > red && green > blue && green >= 300)
            return Color.Green; // Verde

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

    public Color GetColorForPoz(int poz) {
        return artifacte[poz];
    }

    public void SetArtifacts() {
        artifacte[0] = Color.Green;
        artifacte[1] = Color.Purple;
        artifacte[2] = Color.Purple;
        artifactCount = 3;
        CalculateFrequency();
    }

    public void SetPozition(double pos) {
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
        // Auto-reset once at the start (Multi-phase)
        if (needsInitialReset) {
            PerformReset();
            return; // Skip ball detection until reset phases are complete
        }

        // Normal operation - detect balls
        ArtifacteIndx();
    }
}