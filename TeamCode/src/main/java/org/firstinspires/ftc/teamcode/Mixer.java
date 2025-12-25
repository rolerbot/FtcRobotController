package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Mixer implements Subsystem{
    public Servo ServoMixer1 = null;
    public Servo ServoMixer2 = null;
    private final Intake intake;
    private final ElapsedTime runtime = new ElapsedTime();
    private boolean isRunning = false;
    private final double initialPosition = 0.0206;
    private double currentPosition = initialPosition;
    private double offsetPosition = 0.3834 / 2;
    ColorSensor cSensor;
    Color[] artifacte = new Color[3];
    int lenPozitii = 0;
    public Mixer(Intake intake){
        this.intake = intake;
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

    public void StartTimer()
    {
        ResetTimer();
        runtime.startTime();
    }

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
    double GetCurrentPosition(){
        return currentPosition;
    }
    void IncrementPosition()
    {
        if(this.currentPosition + offsetPosition > 1.0 || this.currentPosition + offsetPosition < 0)
            this.offsetPosition *= (-1);
        this.currentPosition += this.offsetPosition;
    }
    void NextPosition()
    {
        IncrementPosition();
        ServoMixer1.setPosition(this.GetCurrentPosition());
        ServoMixer2.setPosition(this.GetCurrentPosition());
    }
    void ArtifacteIndx()
    {
        if (!isRunning && lenPozitii < 3)
        {
            StartTimer();
            Color detectedColor = Culoare(cSensor);
            if(detectedColor != Color.None)
            {
                intake.SetMotorPower(0.3);
                artifacte[lenPozitii++] = detectedColor;
                isRunning = true;
            }
        }
        else if (isRunning && GetTimerElapsed() > 0.7)
        {
            isRunning = false;
            if (lenPozitii == 3)
            {
                //lenPozitii--;
                //IncrementPosition();
                intake.SetMotorPower(0.3);
            }
            else
            {
                IncrementPosition();
                NextPosition();
            }
        }
    }
    private Color Culoare(ColorSensor cSensor)
    {
        int red = cSensor.red();
        int green = cSensor.green();
        int blue = cSensor.blue();

        if (red > green && blue > green && red > 300 && blue > 300)
            return Color.Purple; // Mov

        if (green > red && green > blue && green > 450)
            return Color.Green; // Verde

        return Color.None;
    }

    public boolean IsEmpty(){
        return lenPozitii == 0;
    }

    public int RemoveArtifact(){
        if (this.IsEmpty()){
            return -1;
        }
        lenPozitii--;
        return lenPozitii;
    }

    public double GetTimerElapsed(){
        return runtime.seconds();
    }

    public void Run(){
        ArtifacteIndx();
    }
}
