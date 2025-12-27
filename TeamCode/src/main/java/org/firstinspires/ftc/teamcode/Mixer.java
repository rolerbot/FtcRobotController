package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.Debug;
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
    private boolean isWaitingForBall = false;
    private final double initialPosition = 0.0206;
    private double currentPosition = initialPosition;
    private double offsetPosition = 0.3834 / 2;
    private int countPurple = 0;
    private int countGreen = 0;
    ColorSensor cSensor;
    public Color[] artifacte= {Color.None, Color.None, Color.None};

    int lenPozitii = 0;
    public Mixer(TelemetryCustom lg, Intake intake)
    {
        this.intake = intake;
        this.logger = lg;
    }
    public void ResetTimer(){
        runtime.reset();
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
    public void StartTimer() {ResetTimer(); runtime.startTime();}
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
        lenPozitii = 0;
        isRunning = false;
        isWaitingForBall = false;
        if(offsetPosition < 0)
            offsetPosition *= (-1);
        for (int i = 0; i <= 2; i++)
            artifacte[i] = Color.None;
    }
    void ArtifacteIndx()
    {
        if (!intake.IsStopped() && !isRunning && !isWaitingForBall && lenPozitii < 3)
        {
            StartTimer();
            Color detectedColor = Culoare(cSensor);

            if(detectedColor != Color.None)
            {
                logger.Log("Detected Color", Utils.ColorToString(detectedColor));
                logger.Log("Nr. Bile in mixer", lenPozitii);
                artifacte[lenPozitii++] = detectedColor;
                isRunning = true;
                isWaitingForBall = true;
                int index = 0;
                for (Color col : artifacte) {
                    if (col != null) { // Safety check
                        logger.Log(String.format("Bila mixer pozitie %d", index), Utils.ColorToString(col));
                    }
                    index++;
                }
            }
        }
        else if (isRunning && isWaitingForBall && GetTimerElapsed() > 0.2 && isWaitingForBall) //&&iswaitingforball
        {
            isWaitingForBall = false;
            if (lenPozitii == 3)
            {
                 intake.SetMotorPower(0.3);
                    CalculateFrequency();
            }
            else
            {
                IncrementPosition();
                NextPosition();
            }
        }
        else if (isRunning && !isWaitingForBall && GetTimerElapsed() > 0.4)
        {
            isRunning = false;
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
    public boolean IsEmpty(){return lenPozitii == 0;}
    public void RemoveArtifact()
    {
        if (this.IsEmpty())
            return;
        lenPozitii--;
        artifacte[lenPozitii] = Color.None;
    }

    public void SetPozition(double pos)
    {
        if (pos < 0.0 || pos > 1.0)
            return;
        ServoMixer1.setPosition(pos);
        ServoMixer2.setPosition(pos);
    }
    public double GetTimerElapsed(){return runtime.seconds();}
    public void Run(){ArtifacteIndx();}
}
