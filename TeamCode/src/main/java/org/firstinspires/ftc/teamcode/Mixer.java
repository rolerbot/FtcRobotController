package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Mixer implements Subsystem{
    public Servo ServoMixer1 = null;
    public Servo ServoMixer2 = null;
    private TelemetryCustom logger;
    private final Intake intake;
    private final ElapsedTime runtime = new ElapsedTime();
    private boolean isRunning = false;
    private boolean waitForBall = false;
    private final double initialPosition = 0.0206;
    private double currentPosition = initialPosition;
    private double offsetPosition = 0.3834 / 2;
    private int countPurple = 0;
    private int countGreen = 0;
    ColorSensor cSensor;
    public Color[] artifacte= {Color.None, Color.None, Color.None};
    int artifactCount = 0;
    public Mixer(TelemetryCustom lg, Intake intake)
    {
        this.intake = intake;
        this.logger = lg;
    }
    public void LinkComponents(HardwareMap hardwareMap)
    {

        cSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        ServoMixer1 = hardwareMap.get(Servo.class, "ServoMixer1");
        ServoMixer2 = hardwareMap.get(Servo.class, "ServoMixer2");
    }
    public void Initialize(HardwareMap hwMap){
        LinkComponents(hwMap);
        ServoMixer1.setDirection(Servo.Direction.REVERSE);
        ServoMixer2.setDirection(Servo.Direction.REVERSE);
        ServoMixer1.setPosition(initialPosition);
        ServoMixer2.setPosition(initialPosition);
    }
    public void StartTimer() {runtime.reset(); runtime.startTime();}
    public int GetColorBlue()
    {
        return cSensor.blue();
    }

    public int GetColorGreen()
    {
        return cSensor.green();
    }

    public int GetColorRed()
    {
        return cSensor.red();
    }
    public double GetCurrentPosition(){
        return currentPosition;
    }

    public void ReverseIncrement()
    {
        offsetPosition *= (-1);
    }
    private void IncrementPosition()
    {
        if(this.currentPosition + this.offsetPosition > 1.0 || this.currentPosition + this.offsetPosition < -1.0)
            this.offsetPosition *= (-1);
        this.currentPosition += this.offsetPosition;
    }
    ///  Rotates the Mixer 60 degrees to a side, depending on servo limits.
    void NextPosition()
    {
        IncrementPosition();
        ServoMixer1.setPosition(this.GetCurrentPosition());
        ServoMixer2.setPosition(this.GetCurrentPosition());
    }
    public double GetServoPosition() {return ServoMixer1.getPosition();}

    public void ResetServoPosition()
    {
        ServoMixer1.setPosition(initialPosition);
        ServoMixer2.setPosition(initialPosition);
        currentPosition = initialPosition;
        artifactCount = 0;
        isRunning = false;
        if(offsetPosition < 0)
            offsetPosition *= (-1);
        for (int i = 0; i <= 2; i++)
            artifacte[i] = Color.None;
    }

    void ArtifacteIndx()
    {
        if (!intake.IsStopped() && !isRunning && !waitForBall && artifactCount < 3)
        {
            StartTimer();
            Color detectedColor = Culoare(cSensor);

            if(detectedColor != Color.None)
            {
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
        }
        else if (isRunning && !waitForBall && GetTimerElapsed() > 0.3 && GetTimerElapsed() < 0.75)
        {
            waitForBall = true;
            if (artifactCount == 3)
                 intake.SetMotorPower(0.3);
            else
            {
                IncrementPosition();
                NextPosition();
            }
        }
        else if(isRunning && waitForBall && GetTimerElapsed() >= 0.75)
        {
            isRunning = false;
            waitForBall = false;
            CalculateFrequency();
        }
    }

    private void CalculateFrequency()
    {
        countPurple = 0;
        countGreen = 0;
        for (Color col : artifacte)
        {
            if (col == Color.Purple)
                countPurple++;
            else if (col == Color.Green)
                countGreen++;
        }
        logger.Log("Nr. bile mov", countPurple);
        logger.Log("Nr. bile verzi", countGreen);
    }

    public int GetColorPosition(Color color)
    {
        logger.Log("Searching for", Utils.ColorToString(color));
        for (int i = 0; i < 3; i++)
        {
            logger.Log(String.format("Slot %d", i), Utils.ColorToString(artifacte[i]));
            if (artifacte[i] != Color.None && artifacte[i] == color)
            {
                logger.Log("Found at position", i);
                return i;
            }
        }
        logger.Log("Color not found", -1);
        return -1;
    }

    public int GetFirstAvailablePosition()
    {
        for(int i = 0; i < artifacte.length; i++)
        {
            if(artifacte[i] != Color.None)
                return i;
        }
        return -1;
    }

    public int GetCountGreen(){return countGreen;}
    public int GetCountPurple(){return countPurple;}
    private Color Culoare(ColorSensor cSensor)
    {
        int red = cSensor.red();
        int green = cSensor.green();
        int blue = cSensor.blue();

        if (green < 200 && red > blue && red > green && blue > green)
            return Color.Purple; // Mov

        if (green > red && green > blue && green > 300)
            return Color.Green; // Verde

        return Color.None;
    }
    public boolean IsEmpty()
    {
        return artifactCount == 0;
    }
    public void RemoveArtifact(int position)
    {
        if(position >= 0 && position < artifacte.length)
        {
            artifacte[position] = Color.None;
            artifactCount--;
            CalculateFrequency();
        }
    }

    public void SetArtifacts()
    {
        artifacte[0] = Color.Green;
        artifacte[1] = Color.Purple;
        artifacte[2] = Color.Purple;
        artifactCount = 3;
        CalculateFrequency();
    }

    public void SetPozition(double pos)
    {
        if (pos < 0.0 || pos > 1.0)
            return;
        ServoMixer1.setPosition(pos);
        ServoMixer2.setPosition(pos);
    }
    public double GetTimerElapsed(){return runtime.seconds();}
    public int GetArtifactCount(){return artifactCount;}
    public void Run(){ArtifacteIndx();}
}
