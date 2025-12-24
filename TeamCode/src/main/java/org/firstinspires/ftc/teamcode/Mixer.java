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
    private double initialPosition = 0.0206;
    private double currentPosition = initialPosition;
    private double offsetPosition = 0.3834 / 2;
    // double[] pozitiiIndx = {0.0206, 0.404, 0.7856}; //3 pozitii
    ColorSensor cSensorSt, cSensorDr;
    Color[] artifacte = new Color[3];
    int lenPozitii = 0;

    public Mixer(Intake intake){
        this.intake = intake;
    }
    public void ResetTimer(){
        runtime.reset();
    }
    public void LinkComponents(HardwareMap hardwareMap){
        ServoMixer1 = hardwareMap.get(Servo.class, "ServoMixer1");
        ServoMixer2 = hardwareMap.get(Servo.class, "ServoMixer2");
    }
    public void Initialize(HardwareMap hwMap){
        LinkComponents(hwMap);
        ServoMixer1.setDirection(Servo.Direction.FORWARD);
        ServoMixer2.setDirection(Servo.Direction.FORWARD);
        ServoMixer1.setPosition(initialPosition);
        ServoMixer2.setPosition(initialPosition);
    }
    double GetCurrentPosition(){
        return currentPosition;
    }
    void IncrementPosition(){
        currentPosition += offsetPosition;
    }
    void NextPosition(){
        mixer.IncrementPosition();
        mixer.SetPosition(mixer.GetCurrentPosition());
    }
    void ArtifacteIndx()
    {
        if (intake.IsStopped() && runtime.seconds() == 0)
        {
            runtime.startTime();
            Color colorLeft = Culoare(cSensorSt);
            //Color colorRight = Culoare(cSensorDr);
            artifacte[lenPozitii++] = colorLeft;
        }
        else if (runtime.seconds() > 1)
        {
            if (lenPozitii == 3)
            {
                intake.StopMotor();
            }
            else
            {
                mixer.IncrementPosition();
                mixer.IncrementPosition(); // dublu increment pentru feed bila
                ServoMixer1.setPosition(GetCurrentPosition());
            }
            runtime.reset();
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
        if (mixer.IsEmpty()){
            telemetry.addData("Mixer gol!", "");
            return -1;
        }
        lenPozitii--;
        // ServoMixer1.setPosition(pozitiiIndx[lenPozitii]);
        return lenPozitii;
    }

    public double GetTimerElapsed(){
        return runtime.seconds();
    }

    public int NextPosition(){
        return lenPozitii - 1;
    }

    public void SetPosition(int position){
        if (position < 0 || position > 2){
            telemetry.addData("Pozitie invalida", position);
            return;
        }
        lenPozitii = position;
        ServoMixer1.setPosition(pozitiiIndx[position]);
    }

    public void Run(){
        ArtifacteIndx();
    }
}
