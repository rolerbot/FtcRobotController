package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

enum Color {None,Purple, Green};
public class Mixer implements Subsystem{
    public Servo ServoMixer1 = null;
    public Servo ServoMixer2 = null;
    private final Intake intake;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isRunning = false;
    private double initialPosition = 0.0206;
    private double currentPosition = initialPosition;
    private double offsetPosition = 0.3834 / 2;
    ColorSensor cSensor;
    Color[] artifacte = new Color[3];
    int lenPozitii = 0;
    private boolean Direction = true; // true - up, false - down

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
        if(this.currentPosition + offsetPosition > 1.0 || this.currentPosition + offsetPosition < -1)
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
        if (intake.IsStopped() && !isRunning && lenPozitii < 3)
        {
            StartTimer();
            Color colorLeft = Culoare(cSensor);
            if(colorLeft != Color.None)
            {
                artifacte[lenPozitii++] = colorLeft;
                isRunning = true;
            }
        }
        else if (GetTimerElapsed() > 0.7 && isRunning)
        {
            isRunning = false;
            if (lenPozitii == 3)
            {
                //lenPozitii--;
                //IncrementPosition();
                intake.StopMotor();
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

        if (red > green && blue > green && red > 100 && blue > 100)
            return Color.Purple; // Mov

        if (green > red && green > blue && green > 100)
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
