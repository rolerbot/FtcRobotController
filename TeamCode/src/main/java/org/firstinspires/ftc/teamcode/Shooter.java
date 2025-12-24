package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter implements Subsystem{
    private ButtonReader Aruncare;
    private final GamepadEx ct1;
    private final double initialPosition = 0.0506;
    private final double finalPosition = 0.6134;
    private final double currentPosition = initialPosition;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isShooting = false;
    // private final double[] pozitiiAruncare = {0.225, 0.6117, 1}; // 3 pozitii aruncare
    private final Mixer mixer;
    private final Intake intake;

    public Servo ServoRidicare = null;
    /// Motor Aruncare
    public  DcMotorEx MotorAruncare = null;
    public Shooter(Mixer mixer,Intake intk,GamepadEx ct1)
    {
        this.ct1 = ct1;
        this.mixer = mixer;
        this.intake = intk;
    }
     public void LinkComponents(HardwareMap hardwareMap)
     {
        Aruncare = new ButtonReader(ct1, GamepadKeys.Button.A);
        ServoRidicare = hardwareMap.get(Servo.class, "ServoRidicare");
        MotorAruncare = hardwareMap.get(DcMotorEx.class, "MotorAruncare");
    }
    public void Initialize(HardwareMap hwMap)
    {
        LinkComponents(hwMap);
        MotorAruncare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorAruncare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorAruncare.setDirection(DcMotorSimple.Direction.FORWARD);

        ServoRidicare.setDirection(Servo.Direction.FORWARD);
        ServoRidicare.setPosition(initialPosition);
    }
    public void Run(){
        AruncareArtifacte();
    }
    public void SetPositionLever(double position){
        ServoRidicare.setPosition(position);
    }
    //public double GetCurrentPositionLever(){return currentPosition;}
    public double GetPositionLever(){
        return ServoRidicare.getPosition();
    }
    private void ResetTimer(){
        runtime.reset();
    }
    void AruncareArtifacte() {
        Aruncare.readValue();

        if (Aruncare.wasJustPressed() && intake.IsStopped() && !isShooting)//// //&& mixer.Empty();
        {
            isShooting = true;
            telemetry.addData("ARUNC la pozitia: ", mixer.GetCurrentPosition());

            ResetTimer();

            mixer.NextPosition(); // pregateste sa traga
            SetPositionLever(finalPosition); // ridica

            telemetry.addData("AM RIDICAT la pozitia: ", GetPositionLever());
        }
        if (isShooting && runtime.seconds() > 1) {
            isShooting = false;
            telemetry.addData("Coboara la pozitia: ", initialPosition);
            SetPositionLever(initialPosition); // coboara
            mixer.NextPosition();
            mixer.RemoveArtifact();
            ResetTimer();
        }
    }
}
